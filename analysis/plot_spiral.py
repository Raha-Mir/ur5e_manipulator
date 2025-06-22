#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from scipy.signal import savgol_filter
import pickle
from pathlib import Path

# Load data
log_path = Path.home() / 'ur5e_logs/ur5e_spiral_cartesian.pkl'
with open(log_path, 'rb') as f:
    data = pickle.load(f)

actual_pos = np.array(data['actual_positions'])
desired_pos = np.array(data['desired_positions'])
time = np.array(data['time'])

print("Actual shape:", actual_pos.shape)
print("Desired shape:", desired_pos.shape)

# Multi-stage trajectory smoothing
def filter_trajectory(trajectory, window=9, polyorder=3, smoothness=0.01, num_points=1000):
    """Apply Savitzky-Golay + B-spline smoothing to a 3D trajectory."""
    if trajectory.shape[0] < 4:
        print("[WARN] Too few points to smooth.")
        return trajectory

    try:
        # Savitzky-Golay filter
        filtered = savgol_filter(trajectory, window_length=min(window, trajectory.shape[0] // 2 * 2 + 1),
                                 polyorder=polyorder, axis=0, mode='interp')

        # B-spline
        tck, _ = splprep([filtered[:, 0], filtered[:, 1], filtered[:, 2]], s=smoothness)
        u_fine = np.linspace(0, 1, num_points)
        x_smooth, y_smooth, z_smooth = splev(u_fine, tck)
        return np.stack([x_smooth, y_smooth, z_smooth], axis=1)

    except Exception as e:
        print(f"[ERROR] Smoothing failed: {e}")
        return trajectory

# Smoothing toggle
enable_smoothing = True
actual_smooth = filter_trajectory(actual_pos, smoothness=0.002) if enable_smoothing else actual_pos
desired_smooth = filter_trajectory(desired_pos, smoothness=0.0005) if enable_smoothing else desired_pos

# Color map
norm = Normalize(vmin=time.min(), vmax=time.max())
colors = cm.plasma(norm(np.linspace(time.min(), time.max(), len(actual_smooth))))

# Create 3D plot
fig = plt.figure(figsize=(16, 12))
ax = fig.add_subplot(111, projection='3d')

# Plot actual trajectory with gradient
for i in range(len(actual_smooth) - 1):
    ax.plot(actual_smooth[i:i+2, 0], actual_smooth[i:i+2, 1], actual_smooth[i:i+2, 2],
            color=colors[i], linewidth=2.5)

# Plot desired trajectory
ax.plot(desired_smooth[:, 0], desired_smooth[:, 1], desired_smooth[:, 2],
        'k--', linewidth=1.5, label='Desired Path')

# Markers
ax.scatter(*actual_pos[0], color='green', s=80, marker='o', label='Start')
ax.scatter(*actual_pos[-1], color='red', s=80, marker='o', label='End')

# Projection lines
for i in np.linspace(0, len(actual_pos) - 1, 25, dtype=int):
    ax.plot([actual_pos[i, 0]] * 2,
            [actual_pos[i, 1]] * 2,
            [0, actual_pos[i, 2]],
            'gray', linestyle='--', alpha=0.25)

# Aspect ratio
max_range = np.ptp(actual_pos, axis=0).max() * 0.5
mid = actual_pos.mean(axis=0)
ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

# Labels
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_zlabel('Z (m)', fontsize=12)
ax.set_title('UR5e End-Effector Spiral Trajectory', fontsize=16, pad=20)

# Unified box styling
box_style = dict(facecolor='white', edgecolor='gray', alpha=0.85)

# Spiral info box (aligned with legend)
spiral_info = (
    f"Spiral Parameters\n"
    f"• Radius: 0.10 m\n"
    f"• Rise/Rev: 0.05 m\n"
    f"• Revolutions: 3\n"
    f"• Total Rise: {3*0.05:.2f} m"
)
ax.text2D(0.73, 0.93, spiral_info, transform=ax.transAxes, fontsize=10, bbox=box_style)

# Legend with box matching info style
legend = ax.legend(loc='upper left', fontsize=10, frameon=True)
legend.get_frame().set_facecolor('white')
legend.get_frame().set_edgecolor('gray')
legend.get_frame().set_alpha(0.85)

# Colorbar
sm = cm.ScalarMappable(cmap='plasma', norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, shrink=0.5, pad=0.1)
cbar.set_label('Time (s)', fontsize=10)

# View and save
ax.view_init(elev=30, azim=-45)
plt.tight_layout()
output_path = Path.home() / 'ur5e_logs/spiral_trajectory_3d_advanced.png'
plt.savefig(str(output_path), dpi=300)
plt.show()
