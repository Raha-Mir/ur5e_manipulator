
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import pinocchio as pin
from pathlib import Path
import pickle

class UR5eHighAccCircleCartesian:
    def __init__(self):
        urdf_path = Path.home() / "new_ws/src/ur5e_manipulator/urdf/my_ur5e.urdf"
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.data_des = self.model.createData()  # Separate data for desired state

        # Get frame ID once during initialization
        self.frame_id = self.model.getFrameId("tool0")

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # Joint limits for clamping
        self.joint_limits = {
            'shoulder_pan_joint': [-3.14, 3.14],
            'shoulder_lift_joint': [-6.28, 6.28],
            'elbow_joint': [-3.14, 3.14],
            'wrist_1_joint': [-6.28, 6.28],
            'wrist_2_joint': [-6.28, 6.28],
            'wrist_3_joint': [-6.28, 6.28]
        }

        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)
        self.prev_v_filt = np.zeros(6)
        self.prev_time = None
        self.received_first_state = False

        # Reduced control gains for stability
        self.Kp = np.diag([80, 80, 90, 60, 60, 60])
        self.Kd = np.diag([15, 15, 15, 15, 15, 15])
        self.max_torque = 100.0

        # Filter params
        self.alpha_v = 0.3
        self.alpha_a = 0.3
        self.alpha_vd = 0.3
        self.alpha_ad = 0.3

        self.v_filt = np.zeros(6)
        self.a_filt = np.zeros(6)
        self.v_desired_filt = np.zeros(6)
        self.a_desired_filt = np.zeros(6)

        # Circle settings
        self.center = np.array([0.4, 0.0, 0.3])
        self.radius = 0.1
        self.period = 6.0

        # Pre-motion settings
        self.pre_motion_done = False
        self.pre_motion_duration = 3.0
        self.t_pre_start = None
        self.q_start = None
        
        # Initialize q_pre_target using IK for circle start position
        R_target = np.diag([1, -1, -1])
        circle_start = np.array([self.center[0] + self.radius, self.center[1], self.center[2]])
        self.q_pre_target = self.compute_ik(circle_start, R_target)

        self.q_des_prev = np.zeros(6)
        self.v_des_prev = np.zeros(6)
        self.start_time = None  # Initialize properly

        # Logs
        self.q_log_actual = []
        self.q_log_desired = []
        self.v_log_actual = []
        self.v_log_desired = []
        self.a_log_actual = []
        self.a_log_desired = []
        self.tau_log = []
        self.time_log = []   
        self.actual_positions = []
        self.desired_positions = []

        rospy.Subscriber("/ur5e/joint_states", JointState, self.joint_state_callback)
        self.pub = rospy.Publisher("/ur5e/ur5e_id_controller/command", Float64MultiArray, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

    def clamp_joints(self, q):
        """Clamp joints within safe limits"""
        for i, name in enumerate(self.joint_names):
            low, high = self.joint_limits[name]
            if q[i] < low:
                q[i] = low
            elif q[i] > high:
                q[i] = high
        return q

    def compute_ik(self, target_pos, target_rot, q_init=None, max_iters=100, tol=1e-4):
        if q_init is None:
            q = self.q.copy()
        else:
            q = q_init.copy()

        for i in range(max_iters):
            pin.forwardKinematics(self.model, self.data_des, q)
            pin.updateFramePlacements(self.model, self.data_des)
            oMf = self.data_des.oMf[self.frame_id]
            x_curr = oMf.translation
            R_curr = oMf.rotation

            dx = target_pos - x_curr
            dR = target_rot @ R_curr.T
            dr = 0.5 * (np.cross(R_curr[:, 0], target_rot[:, 0]) +
                        np.cross(R_curr[:, 1], target_rot[:, 1]) +
                        np.cross(R_curr[:, 2], target_rot[:, 2]))

            err = np.hstack((dx, dr))
            if np.linalg.norm(err) < tol:
                break

            J = pin.computeFrameJacobian(self.model, self.data_des, q, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            lambda_reg = 1e-6
            dq = np.linalg.solve(J.T @ J + lambda_reg * np.eye(6), J.T @ err)
            q = self.clamp_joints(q + dq)

        q = self.clamp_joints(q)
        pin.forwardKinematics(self.model, self.data_des, q)
        pin.updateFramePlacements(self.model, self.data_des)
        oMf = self.data_des.oMf[self.frame_id]
        pos_error = np.linalg.norm(target_pos - oMf.translation)
        if pos_error > 0.01:
            rospy.logwarn(f"IK position error: {pos_error:.4f}m after {i+1} iterations")
        else:
            rospy.loginfo(f"IK converged in {i+1} iterations with error: {pos_error:.6f}m")

        return q

    def compute_desired_pose(self, t):
        omega = 2 * np.pi / self.period
        x = self.center[0] + self.radius * np.cos(omega * t)
        z = self.center[2] + self.radius * np.sin(omega * t)
        pos = np.array([x, self.center[1], z])
        R = np.diag([1, -1, -1])
        return pos, R

    def minimum_jerk(self, t, T):
        tau = np.clip(t / T, 0.0, 1.0)
        return 10 * tau**3 - 15 * tau**4 + 6 * tau**5

    def joint_state_callback(self, msg):
        if not msg.position or len(msg.position) < 6:
            return

        q_in = np.array(msg.position)
        if np.linalg.norm(q_in) < 1e-3:
            return

        now = rospy.Time.now()
        if self.prev_time is None:
            self.prev_time = now
        dt = (now - self.prev_time).to_sec()
        dt = max(dt, 1e-5)
        self.prev_time = now

        # Initialize t_elapsed for logging
        t_elapsed = 0.0

        for i, joint in enumerate(self.joint_names):
            if joint in msg.name:
                idx = msg.name.index(joint)
                self.q[i] = msg.position[idx]
                self.v[i] = msg.velocity[idx] if msg.velocity and len(msg.velocity) > idx else 0.0

        if not self.received_first_state:
            self.received_first_state = True
            self.q_start = self.q.copy()
            self.q_des_prev = self.q_start.copy()
            self.t_pre_start = now
            rospy.loginfo("Received first joint state. Starting pre-motion phase.")
            return

        # Filtered actual velocity
        raw_velocity = self.v.copy()
        self.v_filt = self.alpha_v * raw_velocity + (1 - self.alpha_v) * self.v_filt

        # Acceleration estimate
        if dt > 1e-4:
            a_measured = (self.v_filt - self.prev_v_filt) / dt
            self.prev_v_filt = self.v_filt.copy()
            self.a_filt = self.alpha_a * a_measured + (1 - self.alpha_a) * self.a_filt
        else:
            a_measured = np.zeros(6)

        if not self.pre_motion_done:
            t_pre_elapsed = (now - self.t_pre_start).to_sec()
            s = self.minimum_jerk(t_pre_elapsed, self.pre_motion_duration)

            q_des = (1 - s) * self.q_start + s * self.q_pre_target
            if dt > 1e-4:
                v_desired = (q_des - self.q_des_prev) / dt
                a_desired = (v_desired - self.v_des_prev) / dt
            else:
                v_desired = np.zeros(6)
                a_desired = np.zeros(6)

            if s >= 1.0:
                self.pre_motion_done = True
                self.start_time = rospy.Time.now()
                rospy.loginfo("Pre-motion phase done. Starting circular motion.")
                # Set t_elapsed to 0 at transition
                t_elapsed = 0.0
        else:
            # Calculate elapsed time for circular motion
            t_elapsed = (now - self.start_time).to_sec()
            x_des, R_des = self.compute_desired_pose(t_elapsed)

            # Start IK from previous desired position
            q_des = self.q_des_prev.copy()
            
            # Perform IK iterations with clamping
            for _ in range(20):
                pin.forwardKinematics(self.model, self.data_des, q_des)
                pin.updateFramePlacements(self.model, self.data_des)
                oMf = self.data_des.oMf[self.frame_id]
                x_curr = oMf.translation
                R_curr = oMf.rotation

                dx = x_des - x_curr
                dR = R_des @ R_curr.T
                dr = 0.5 * (np.cross(R_curr[:,0], R_des[:,0]) +
                            np.cross(R_curr[:,1], R_des[:,1]) +
                            np.cross(R_curr[:,2], R_des[:,2]))

                err = np.hstack((dx, dr))

                J = pin.computeFrameJacobian(self.model, self.data_des, q_des, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                lambda_reg = 1e-6
                dq = np.linalg.solve(J.T @ J + lambda_reg * np.eye(6), J.T @ err)
                q_des = self.clamp_joints(q_des + dq)
            
            # Final clamping
            q_des = self.clamp_joints(q_des)
            
            # Compute desired velocity/acceleration
            if dt > 1e-4:
                v_desired = (q_des - self.q_des_prev) / dt
                a_desired = (v_desired - self.v_des_prev) / dt
            else:
                v_desired = np.zeros(6)
                a_desired = np.zeros(6)

        # Update desired state history
        self.q_des_prev = q_des.copy()
        self.v_des_prev = v_desired.copy()
        
        # Filter desired velocity/acceleration
        self.v_desired_filt = self.alpha_vd * v_desired + (1 - self.alpha_vd) * self.v_desired_filt
        self.a_desired_filt = self.alpha_ad * a_desired + (1 - self.alpha_ad) * self.a_desired_filt

        # Compute torque using inverse dynamics
        error = q_des - self.q
        d_error = self.v_desired_filt - self.v_filt
        a_ref = self.a_desired_filt + self.Kp @ error + self.Kd @ d_error

        # Compute torque with RNEA
        tau = pin.rnea(self.model, self.data, self.q, self.v_filt, a_ref)
        tau_saturated = np.clip(tau, -self.max_torque, self.max_torque)

        # Publish control command
        msg_out = Float64MultiArray()
        msg_out.data = tau_saturated.tolist()
        self.pub.publish(msg_out)
        
        # === CRITICAL ADDITIONS FOR 3D PLOTTING ===
        # Compute actual end-effector position
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        oMf_actual = self.data.oMf[self.frame_id]
        actual_pos = oMf_actual.translation
        
        # Compute desired end-effector position
        pin.forwardKinematics(self.model, self.data_des, q_des)
        pin.updateFramePlacements(self.model, self.data_des)
        oMf_desired = self.data_des.oMf[self.frame_id]
        desired_pos = oMf_desired.translation
        
        # Log positions
        self.actual_positions.append(actual_pos.copy())
        self.desired_positions.append(desired_pos.copy())
        # === END CRITICAL ADDITIONS ===

        # Logging - t_elapsed is now properly defined in all cases
        self.q_log_actual.append(self.q.copy())
        self.q_log_desired.append(q_des.copy())
        self.v_log_actual.append(self.v_filt.copy())
        self.v_log_desired.append(self.v_desired_filt.copy())
        self.a_log_actual.append(self.a_filt.copy())
        self.a_log_desired.append(self.a_desired_filt.copy())
        self.tau_log.append(tau_saturated.copy())
        self.time_log.append(t_elapsed)

        rospy.loginfo_throttle(1.0, f"t={t_elapsed:.2f} pre_motion={not self.pre_motion_done} q_act={np.round(self.q, 2)} q_des={np.round(q_des, 2)}")

    def on_shutdown(self):
        min_len = min(
            len(self.time_log),
            len(self.q_log_actual),
            len(self.q_log_desired),
            len(self.v_log_actual),
            len(self.v_log_desired),
            len(self.a_log_actual),
            len(self.a_log_desired),
            len(self.actual_positions),
            len(self.desired_positions)
        )

        data = {
            'time': self.time_log[:min_len],
            'q_actual': np.array(self.q_log_actual[:min_len]),
            'q_desired': np.array(self.q_log_desired[:min_len]),
            'v_actual': np.array(self.v_log_actual[:min_len]),
            'v_desired': np.array(self.v_log_desired[:min_len]),
            'a_actual': np.array(self.a_log_actual[:min_len]),
            'a_desired': np.array(self.a_log_desired[:min_len]),
            'tau': np.array(self.tau_log[:min_len]),
            'actual_positions': np.array(self.actual_positions[:min_len]),
            'desired_positions': np.array(self.desired_positions[:min_len])
        }

        log_dir = Path.home() / 'ur5e_logs'
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / 'ur5e_circular_cartesian.pkl'

        with open(log_path, 'wb') as f:
            pickle.dump(data, f)
        rospy.loginfo(f"Saved circular Cartesian trajectory logs to {log_path}")

if __name__ == '__main__':
    rospy.init_node("ur5e_highacc_circle_cartesian")
    rospy.loginfo("Starting UR5e Cartesian Circular Trajectory Controller with Pre-Motion...")
    controller = UR5eHighAccCircleCartesian()
    rospy.spin()
