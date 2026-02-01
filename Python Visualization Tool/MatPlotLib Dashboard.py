import math
import time
from pathlib import Path

import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from collections import deque
import threading

try:
    from stl import mesh as stl_mesh

    STL_AVAILABLE = True
except Exception:
    stl_mesh = None
    STL_AVAILABLE = False

# --- CONFIGURATION ---
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
MAX_POINTS = 100
UPDATE_INTERVAL_MS = 75

STL_PATH = Path(__file__).parent / "STLS" / "AnthonyHua.stl"
STL_TARGET_SIZE = 2.0
GYRO_SCALE = 1.0
GYRO_DPS_TO_RADS = math.pi / 180.0
GYRO_SMOOTHING = 0.2

# --- THEME ---
THEME = {
    "fig_bg": "#0b1020",
    "ax_bg": "#0f172a",
    "grid": "#25324a",
    "text": "#e2e8f0",
    "muted": "#94a3b8",
    "accent": "#5ecbff",
    "stl": "#ffffff",
    "x": "#ff6b6b",
    "y": "#5ad4e6",
    "z": "#ffd166",
}

plt.rcParams.update(
    {
        "figure.facecolor": THEME["fig_bg"],
        "axes.facecolor": THEME["ax_bg"],
        "axes.edgecolor": THEME["grid"],
        "axes.labelcolor": THEME["text"],
        "xtick.color": THEME["muted"],
        "ytick.color": THEME["muted"],
        "text.color": THEME["text"],
        "grid.color": THEME["grid"],
        "grid.alpha": 0.6,
        "grid.linestyle": "--",
        "grid.linewidth": 0.6,
    }
)


def style_axis(ax, title):
    ax.set_facecolor(THEME["ax_bg"])
    ax.set_title(title, color=THEME["accent"], pad=10, fontsize=12, fontweight="bold")
    ax.tick_params(colors=THEME["muted"])
    for spine in ax.spines.values():
        spine.set_color(THEME["grid"])
    ax.grid(True)
    ax.set_axisbelow(True)


def style_legend(ax):
    legend = ax.legend(loc="upper right", fontsize="x-small", frameon=True)
    legend.get_frame().set_facecolor(THEME["ax_bg"])
    legend.get_frame().set_edgecolor(THEME["grid"])
    for text in legend.get_texts():
        text.set_color(THEME["text"])


def style_axis_3d(ax):
    ax.set_facecolor(THEME["ax_bg"])
    ax.set_title(
        "STL Orientation (Gyro)",
        color=THEME["accent"],
        pad=10,
        fontsize=12,
        fontweight="bold",
    )
    ax.tick_params(colors=THEME["muted"])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        try:
            axis.pane.set_facecolor(THEME["ax_bg"])
            axis.pane.set_edgecolor(THEME["grid"])
        except Exception:
            pass
    ax.grid(False)


def rotation_matrix(angles):
    rx, ry, rz = angles
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def setup_stl_axis(ax):
    global stl_base_vectors, stl_collection
    style_axis_3d(ax)

    if not STL_AVAILABLE:
        ax.text2D(
            0.05,
            0.92,
            "Install numpy-stl to load STL",
            transform=ax.transAxes,
            color=THEME["muted"],
            fontsize=10,
        )
        return

    if not STL_PATH.exists():
        ax.text2D(
            0.05,
            0.92,
            f"STL not found: {STL_PATH}",
            transform=ax.transAxes,
            color=THEME["muted"],
            fontsize=10,
        )
        return

    mesh = stl_mesh.Mesh.from_file(str(STL_PATH))
    vectors = mesh.vectors
    vertices = vectors.reshape(-1, 3)
    center = vertices.mean(axis=0)
    vertices_centered = vertices - center
    max_range = np.ptp(vertices_centered, axis=0).max()
    scale = STL_TARGET_SIZE / max_range if max_range > 0 else 1.0
    stl_base_vectors = (vertices_centered * scale).reshape(vectors.shape)

    stl_collection = Poly3DCollection(
        stl_base_vectors,
        facecolors=THEME["stl"],
        edgecolors=None,
        linewidths=0,  
        shade=True,  
        alpha=1.0,
    )
    ax.add_collection3d(stl_collection)

    extent = np.max(np.abs(stl_base_vectors))
    limit = max(extent * 1.15, 0.5)
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=20, azim=45)


def update_stl_rotation(gx, gy, gz, dt):
    global gyro_smoothed, orientation
    if stl_collection is None or stl_base_vectors is None:
        return

    rates = np.array([gx, gy, gz], dtype=float)
    rates = rates * GYRO_SCALE * GYRO_DPS_TO_RADS
    gyro_smoothed = (1 - GYRO_SMOOTHING) * gyro_smoothed + GYRO_SMOOTHING * rates
    orientation += gyro_smoothed * dt
    orientation = (orientation + math.pi) % (2 * math.pi) - math.pi

    rot = rotation_matrix(orientation)
    rotated = stl_base_vectors.reshape(-1, 3) @ rot.T
    stl_collection.set_verts(rotated.reshape(stl_base_vectors.shape))


# --- DATA BUFFERS ---
gyro_x = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
gyro_y = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
gyro_z = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

accel_x = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
accel_y = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
accel_z = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

x_axis = deque(range(MAX_POINTS), maxlen=MAX_POINTS)

gyro_smoothed = np.zeros(3, dtype=float)
orientation = np.zeros(3, dtype=float)
last_update_time = time.perf_counter()
stl_base_vectors = None
stl_collection = None

# --- SERIAL CONNECTION ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error: {e}")
    exit()


# --- PARSING FUNCTION ---
def parse_line(line):
    # Split the line by the pipe symbol
    parts = line.split("|")

    # DEBUG: Uncomment the next line if the graph is still flat to see what Python sees
    # print(f"Split parts: {parts}")

    # We need at least 13 parts to have all the data
    if len(parts) < 13:
        return None

    try:
        # --- GYRO INDICES ---
        # parts[1] is X, parts[3] is Y, parts[5] is Z
        gx = int(parts[1].strip())
        gy = int(parts[3].strip())
        gz = int(parts[5].strip())

        # --- ACCEL INDICES (FIXED) ---
        # Previous error: I thought these were at 9,11,13.
        # Correct indices are 8, 10, 12 based on your C code.
        ax = float(parts[8].strip())
        ay = float(parts[10].strip())
        az = float(parts[12].strip())

        return gx, gy, gz, ax, ay, az
    except Exception as e:
        # This prints errors to the console so you know if data is bad
        print(f"Parse Error on line: {line}\nReason: {e}")
        return None


# --- UPDATE FUNCTION ---
def update_graph(frame):
    global last_update_time
    now = time.perf_counter()
    dt = max(now - last_update_time, 1e-3)
    last_update_time = now

    while ser.in_waiting > 0:
        try:
            raw_line = ser.readline()
            line = raw_line.decode("utf-8", errors="ignore").strip()

            data = parse_line(line)

            if data:
                gx, gy, gz, ax, ay, az = data

                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)

                accel_x.append(ax)
                accel_y.append(ay)
                accel_z.append(az)

        except Exception as e:
            print(f"Serial Error: {e}")

    if gyro_x:
        gx = gyro_x[-1]
        gy = gyro_y[-1]
        gz = gyro_z[-1]
    else:
        gx = gy = gz = 0.0
    update_stl_rotation(gx, gy, gz, dt)

    # -- Draw Gyro --
    ax1.clear()
    ax1.plot(gyro_x, label="X", color=THEME["x"], linewidth=1.6, alpha=0.95)
    ax1.plot(gyro_y, label="Y", color=THEME["y"], linewidth=1.6, alpha=0.95)
    ax1.plot(gyro_z, label="Z", color=THEME["z"], linewidth=1.6, alpha=0.95)
    ax1.set_ylabel("Degrees/sec")
    ax1.set_xlabel("Time (samples)")
    style_axis(ax1, "Gyroscope Data")
    ax1.tick_params(labelbottom=True)
    style_legend(ax1)

    # -- Draw Accel --
    ax2.clear()
    ax2.plot(accel_x, label="X", color=THEME["x"], linewidth=1.6, alpha=0.95)
    ax2.plot(accel_y, label="Y", color=THEME["y"], linewidth=1.6, alpha=0.95)
    ax2.plot(accel_z, label="Z", color=THEME["z"], linewidth=1.6, alpha=0.95)
    style_axis(ax2, "Accelerometer Data")
    ax2.set_ylabel("g-forces")
    ax2.set_xlabel("Time (samples)")
    style_legend(ax2)


# --- PLOT SETUP ---
fig = plt.figure(figsize=(13, 7))
fig.patch.set_facecolor(THEME["fig_bg"])
grid = fig.add_gridspec(
    2,
    2,
    width_ratios=[1.1, 1.0],
    height_ratios=[1, 1],
    wspace=0.18,
    hspace=0.28,
)

ax1 = fig.add_subplot(grid[0, 0])
ax2 = fig.add_subplot(grid[1, 0], sharex=ax1)
ax3 = fig.add_subplot(grid[:, 1], projection="3d")
setup_stl_axis(ax3)

ani = animation.FuncAnimation(fig, update_graph, interval=UPDATE_INTERVAL_MS)
plt.show()

ser.close()
