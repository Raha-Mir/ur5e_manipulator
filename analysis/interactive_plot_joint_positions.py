#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from scipy.signal import butter, filtfilt, savgol_filter
from pathlib import Path

# === Load UR5e joint data ===
log_path = Path.home() / 'ur5e_logs/ur5e_traj_logs.pkl'
data = dict(np.load(log_path, allow_pickle=True))

time = np.array(data['time'])
q_actual = np.array(data['q_actual'])         # Actual joint positions (N, 6)
q_desired = np.array(data.get('q_desired', q_actual))  # Desired positions (fallback to actual)

# === Time and sampling ===
dt = np.median(np.diff(time))
fs = 1.0 / dt

# === Filtering function ===
def apply_filter(q, cutoff, order, window, poly):
    window = max(5, int(window))
    if window % 2 == 0:
        window += 1
    q_smooth = savgol_filter(q, window_length=window, polyorder=int(poly), axis=0)

    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(int(order), normal_cutoff, btype='low')
    return filtfilt(b, a, q_smooth, axis=0)

# === Numerical derivatives ===
def compute_derivatives(q, dt):
    v = np.gradient(q, dt, axis=0)
    a = np.gradient(v, dt, axis=0)
    return v, a

# === Compute error metrics ===
def compute_metrics(actual, desired, time):
    overshoot = np.max(actual - desired, axis=0)
    error = actual - desired
    steady = np.abs(error) < 0.02  # Threshold for settling
    settling_time = np.zeros(6)
    for i in range(6):
        idx = np.where(steady[:, i])[0]
        settling_time[i] = time[idx[0]] if len(idx) > 0 else np.nan
    rmse = np.sqrt(np.mean(error**2, axis=0))
    return overshoot, settling_time, rmse

# === Initial filter settings ===
init_cutoff = 2.0
init_order = 3
init_window = 41
init_poly = 3
plot_mode = 'Position'

# === Initial processing ===
qf = apply_filter(q_actual, init_cutoff, init_order, init_window, init_poly)
vf, af = compute_derivatives(qf, dt)
overshoot, settling_time, rmse = compute_metrics(qf, q_desired, time)

# === Plot setup ===
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.3, bottom=0.45)

# Plot actual and desired data
actual_lines = [ax.plot(time, qf[:, i], label=f'Joint {i+1} Actual')[0] for i in range(6)]
desired_lines = [ax.plot(time, q_desired[:, i], '--', label=f'Joint {i+1} Desired')[0] for i in range(6)]

ax.set_title("UR5e Joint Data Viewer")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Position [rad]")
ax.legend()
ax.grid(True)

# === UI Controls: Sliders ===
slider_color = 'lightgoldenrodyellow'
s_cutoff = Slider(plt.axes([0.3, 0.35, 0.65, 0.03], facecolor=slider_color), 'Cutoff (Hz)', 0.1, 10.0, valinit=init_cutoff, valstep=0.1)
s_order  = Slider(plt.axes([0.3, 0.30, 0.65, 0.03], facecolor=slider_color), 'Order', 1, 5, valinit=init_order, valstep=1)
s_window = Slider(plt.axes([0.3, 0.25, 0.65, 0.03], facecolor=slider_color), 'Window Size', 5, 201, valinit=init_window, valstep=2)
s_poly   = Slider(plt.axes([0.3, 0.20, 0.65, 0.03], facecolor=slider_color), 'Poly Order', 1, 5, valinit=init_poly, valstep=1)

# === UI Controls: Plot mode ===
radio_ax = plt.axes([0.05, 0.55, 0.2, 0.3], facecolor=slider_color)
radio = RadioButtons(radio_ax, ('Position', 'Velocity', 'Acceleration', 'Error'), active=0)

# === Info box ===
info_ax = plt.axes([0.05, 0.35, 0.2, 0.18])
info_ax.axis('off')
info_txt = info_ax.text(0, 0.9, '', fontsize=10, verticalalignment='top')

# === Update plot function ===
def update_plot():
    cutoff = s_cutoff.val
    order = int(s_order.val)
    window = int(s_window.val)
    poly = int(s_poly.val)

    qf = apply_filter(q_actual, cutoff, order, window, poly)
    vf, af = compute_derivatives(qf, dt)
    ef = qf - q_desired
    overshoot, settling_time, rmse = compute_metrics(qf, q_desired, time)

    info_txt.set_text(
        f"RMSE [rad]:\n{rmse.round(4)}\n\n"
        f"Overshoot [rad]:\n{overshoot.round(4)}\n\n"
        f"Settling [s]:\n{settling_time.round(2)}"
    )

    if plot_mode == 'Position':
        values = qf
        desired = q_desired
        ax.set_ylabel("Position [rad]")
    elif plot_mode == 'Velocity':
        values = vf
        desired = np.gradient(q_desired, dt, axis=0)
        ax.set_ylabel("Velocity [rad/s]")
    elif plot_mode == 'Acceleration':
        values = af
        v_des = np.gradient(q_desired, dt, axis=0)
        desired = np.gradient(v_des, dt, axis=0)
        ax.set_ylabel("Acceleration [rad/s²]")
    elif plot_mode == 'Error':
        values = ef
        desired = None
        ax.set_ylabel("Error [rad]")

    for i in range(6):
        actual_lines[i].set_ydata(values[:, i])
        if desired is not None:
            desired_lines[i].set_ydata(desired[:, i])
        else:
            desired_lines[i].set_ydata(np.full_like(values[:, i], np.nan))  # hide

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()

# === Event handlers ===
def slider_handler(val):
    update_plot()

def radio_handler(label):
    global plot_mode
    plot_mode = label
    update_plot()

# === Bind UI events ===
s_cutoff.on_changed(slider_handler)
s_order.on_changed(slider_handler)
s_window.on_changed(slider_handler)
s_poly.on_changed(slider_handler)
radio.on_clicked(radio_handler)

plt.show()
