#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from matplotlib import cm
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D projection
import pickle
from pathlib import Path

# Load data
log_path = Path.home() / 'ur5e_logs/ur5e_circular_cartesian.pkl'
with open(log_path, 'rb') as f:
    data = pickle.load(f)

actual_pos = np.array(data['actual_positions'])
desired_pos = np.array(data['desired_positions'])
time = np.array(data['time'])

# Smooth trajectories using Savitzky-Golay filter
window = 31 if len(time) >= 31 else len(time) // 2 * 2 + 1  # Must be odd
actual_smoothed = savgol_filter(actual_pos, window_length=window, polyorder=3, axis=0)
desired_smoothed = savgol_filter(desired_pos, window_length=window, polyorder=3, axis=0)

# Generate ideal circle
center = np.array([0.4, 0.0, 0.3])
radius = 0.1
period = 6.0
theta = 2 * np.pi * time / period
ref_x = center[0] + radius * np.cos(theta)
ref_y = np.full_like(ref_x, center[1])
ref_z = center[2] + radius * np.sin(theta)

# Create figure
fig = plt.figure(figsize=(14, 11))
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor('whitesmoke')

# Create color gradient based on time
norm = Normalize(vmin=time.min(), vmax=time.max())
colors = cm.viridis(norm(time))

# Plot actual path with gradient
for i in range(len(actual_smoothed) - 1):
    ax.plot(actual_smoothed[i:i+2, 0],
            actual_smoothed[i:i+2, 1],
            actual_smoothed[i:i+2, 2],
            color=colors[i], linewidth=2.5)

# Plot other paths
ax.plot(desired_smoothed[:, 0], desired_smoothed[:, 1], desired_smoothed[:, 2],
        'g--', linewidth=1.5, label='Desired Path')
ax.plot(ref_x, ref_y, ref_z,
        'r-', linewidth=1.2, alpha=0.6, label='Ideal Circle')

# Plot key points
ax.scatter(*center, color='k', s=50, label='Circle Center')
ax.scatter(*actual_smoothed[0], color='limegreen', s=50, label='Start')
ax.scatter(*actual_smoothed[-1], color='red', s=50, label='End')

# Compute final Euclidean error
final_error = np.linalg.norm(actual_smoothed[-1] - desired_smoothed[-1])

# Labels and aesthetics
ax.set_title('UR5e End-Effector 3D Trajectory', fontsize=16, pad=20)
ax.set_xlabel('X [m]', fontsize=12, labelpad=10)
ax.set_ylabel('Y [m]', fontsize=12, labelpad=10)
ax.set_zlabel('Z [m]', fontsize=12, labelpad=10)
ax.view_init(elev=25, azim=-60)
ax.set_box_aspect([1, 1, 1])
ax.grid(True)
ax.legend(fontsize=10, loc='upper right')

# Annotations
info = (
    f"Circle:\n  Center: {center.tolist()}\n  Radius: {radius:.2f} m\n  Period: {period:.1f} s\n\n"
    f"Final Pos Error: {final_error:.4f} m"
)
ax.text2D(0.02, 0.98, info, transform=ax.transAxes,
          fontsize=10, verticalalignment='top',
          bbox=dict(facecolor='white', alpha=0.85, boxstyle='round'))

# Colorbar for time
mappable = cm.ScalarMappable(norm=norm, cmap=cm.viridis)
cbar = fig.colorbar(mappable, ax=ax, pad=0.05, shrink=0.6)
cbar.set_label('Time [s]', fontsize=11)

# Save and show
plt.tight_layout()
plt.savefig(Path.home() / 'ur5e_logs/trajectory_3d_smooth.png', dpi=300, transparent=True)
plt.show()
