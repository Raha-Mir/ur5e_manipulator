#!/usr/bin/env python  

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import pinocchio as pin
from pathlib import Path
import pickle

class UR5eHighAccuracyID:
    def __init__(self):
        urdf_path = Path.home() / "new_ws/src/ur5e_manipulator/urdf/my_ur5e.urdf"
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        self.q = np.zeros(self.model.nq)
        self.v = np.zeros(self.model.nv)
        self.prev_v_filt = np.zeros(6)
        self.prev_time = None
        self.received_first_state = False

        self.q_final = np.array([0.0, -1.57, -1.57, 3.14, -1.57, 0.0])

        #manually tuned gain
        self.Kp = np.diag([80, 80, 90, 60, 60, 60])
        self.Kd = np.diag([15, 15, 15, 15, 15, 15])
        self.max_torque = 100.0

        # Logs
        self.q_log_actual = []
        self.q_log_desired = []
        self.v_log_actual = []
        self.v_log_desired = []
        self.a_log_actual = []
        self.a_log_desired = []
        self.tau_log = [] 
        self.time_log = []

        # EMA smoothing parameters
        self.alpha_v = 0.2
        self.alpha_a = 0.2

        self.v_filt = np.zeros(6)
        self.a_filt = np.zeros(6)

        self.q_start = None
        self.start_time = rospy.Time.now()
        self.T = 6.0

        rospy.Subscriber("/ur5e/joint_states", JointState, self.joint_state_callback)
        self.pub = rospy.Publisher("/ur5e/ur5e_id_controller/command", Float64MultiArray, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

    def compute_quintic_trajectory(self, t, T, q0, qf):
        t = np.clip(t, 0, T)
        tau = t / T
        q = q0 + (qf - q0) * (6*tau**5 - 15*tau**4 + 10*tau**3)
        v = (qf - q0) * (30*tau**4 - 60*tau**3 + 30*tau**2) / T
        a = (qf - q0) * (120*tau**3 - 180*tau**2 + 60*tau) / T**2
        return q, v, a

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

        for i, joint in enumerate(self.joint_names):
            idx = msg.name.index(joint)
            self.q[i] = msg.position[idx]
            self.v[i] = msg.velocity[idx] if msg.velocity else 0.0

        if not self.received_first_state:
            self.received_first_state = True
            self.q_start = self.q[:6].copy()
            self.start_time = now
            rospy.loginfo("Received first joint state. Starting control.")
            return

        t_elapsed = (now - self.start_time).to_sec()

        q_desired, v_desired, a_desired = self.compute_quintic_trajectory(
            t_elapsed, self.T, self.q_start, self.q_final
        ) if t_elapsed <= self.T else (self.q_final.copy(), np.zeros(6), np.zeros(6))

        # Filtered actual velocity
        raw_velocity = self.v[:6].copy()
        self.v_filt = self.alpha_v * raw_velocity + (1 - self.alpha_v) * self.v_filt

        # Acceleration estimate from filtered velocity
        a_measured = (self.v_filt - self.prev_v_filt) / dt
        self.prev_v_filt = self.v_filt.copy()

        # EMA filter on acceleration
        self.a_filt = self.alpha_a * a_measured + (1 - self.alpha_a) * self.a_filt

        # Control law
        error = q_desired - self.q[:6]
        d_error = v_desired - self.v_filt

        a_ref = a_desired + self.Kp @ error + self.Kd @ d_error
        tau = pin.rnea(self.model, self.data, self.q, self.v, a_ref)
        tau_saturated = np.clip(tau[:6], -self.max_torque, self.max_torque)
        self.tau_log.append(tau_saturated.copy())

        msg_out = Float64MultiArray()
        msg_out.data = tau_saturated.tolist()
        self.pub.publish(msg_out)

        # Logging
        self.q_log_actual.append(self.q[:6].copy())
        self.q_log_desired.append(q_desired.copy())
        self.v_log_actual.append(self.v_filt.copy())
        self.v_log_desired.append(v_desired.copy())
        self.a_log_actual.append(self.a_filt.copy())
        self.a_log_desired.append(a_desired.copy())
        self.time_log.append(t_elapsed)
        

        # Throttled logs
        rospy.loginfo_throttle(1.0, f"t={t_elapsed:.2f}  q={np.round(self.q[:6], 2)}")
        rospy.loginfo_throttle(1.0, f"v_filt={np.round(self.v_filt, 2)} a_filt={np.round(self.a_filt, 2)}")
        rospy.loginfo_throttle(1.0, "t=%.2f  q=%s", t_elapsed, np.round(self.q[:6], 2))
        rospy.loginfo_throttle(1.0, "q_des=%s", np.round(self.q_final, 2))
        rospy.loginfo_throttle(1.0, "a_des=%s", np.round(a_ref, 2))
        rospy.loginfo_throttle(1.0, "tau: %s", np.round(tau_saturated, 2))

    def on_shutdown(self):
        min_len = min(
            len(self.time_log),
            len(self.q_log_actual),
            len(self.q_log_desired),
            len(self.v_log_actual),
            len(self.v_log_desired),
            len(self.a_log_actual),
            len(self.a_log_desired)
        )

        data = {
            'time': self.time_log[:min_len],
            'q_actual': np.array(self.q_log_actual[:min_len]),
            'q_desired': np.array(self.q_log_desired[:min_len]),
            'v_actual': np.array(self.v_log_actual[:min_len]),
            'v_desired': np.array(self.v_log_desired[:min_len]),
            'a_actual': np.array(self.a_log_actual[:min_len]),
            'a_desired': np.array(self.a_log_desired[:min_len]),
            'tau': np.array(self.tau_log[:min_len])
        }

        log_dir = Path.home() / 'ur5e_logs'
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / 'ur5e_traj_logs.pkl'

        with open(log_path, 'wb') as f:
            pickle.dump(data, f)
        rospy.loginfo(f"Saved synchronized trajectory logs to {log_path}")

if __name__ == '__main__':
    rospy.init_node("ur5e_highacc_id")
    rospy.loginfo("Starting UR5e High-Accuracy ID Controller...")

    UR5eHighAccuracyID()
    rospy.spin()

