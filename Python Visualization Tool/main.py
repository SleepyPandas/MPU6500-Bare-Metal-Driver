from __future__ import annotations

import math
import pathlib
import random
import re
import threading
import time
from collections import deque

import serial
from nicegui import app, ui

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
MOCK_DATA = False

PLOT_HISTORY = 100
SMOOTHING_WINDOW = 12
UPDATE_INTERVAL = 0.1
MOCK_INTERVAL = 0.02
ROTATION_SCALE = math.radians(20)

COLOR_X = "#00f5ff"
COLOR_Y = "#ff4dd2"
COLOR_Z = "#ffd166"

# Serve static files for 3D models only (assuming 'STLS' folder exists in current dir)
base_path = pathlib.Path(__file__).parent
app.add_static_files("/stls", base_path / "STLS")

# Regex to parse DATA from UART lines
FLOAT_PATTERN = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
LINE_RE = re.compile(
    rf"Gyro:\s*X\s*\|\s*(?P<gx>{FLOAT_PATTERN})\s*\|\s*,\s*"
    rf"Y\s*\|\s*(?P<gy>{FLOAT_PATTERN})\s*\|\s*,\s*"
    rf"Z\s*\|\s*(?P<gz>{FLOAT_PATTERN})\s*\|\s*---\s*\|\s*"
    rf"Accel:\s*X\s*\|\s*(?P<ax>{FLOAT_PATTERN})\s*\|\s*,\s*"
    rf"Y\s*\|\s*(?P<ay>{FLOAT_PATTERN})\s*\|\s*,\s*"
    rf"Z\s*\|\s*(?P<az>{FLOAT_PATTERN})\s*\|",
    re.IGNORECASE,
)

data_lock = threading.Lock()
gyro_data = {
    "x": deque(maxlen=PLOT_HISTORY),
    "y": deque(maxlen=PLOT_HISTORY),
    "z": deque(maxlen=PLOT_HISTORY),
}
accel_data = {
    "x": deque(maxlen=PLOT_HISTORY),
    "y": deque(maxlen=PLOT_HISTORY),
    "z": deque(maxlen=PLOT_HISTORY),
}

serial_lock = threading.Lock()
stop_event = threading.Event()
ser = None
serial_thread = None
serial_started = False


def parse_line(line: str) -> dict[str, float] | None:
    match = LINE_RE.search(line)
    if not match:
        return None
    try:
        return {
            "gx": float(match.group("gx")),
            "gy": float(match.group("gy")),
            "gz": float(match.group("gz")),
            "ax": float(match.group("ax")),
            "ay": float(match.group("ay")),
            "az": float(match.group("az")),
        }
    except (ValueError, TypeError):
        return None


def ingest_line(line: str) -> None:
    parsed = parse_line(line)
    if not parsed:
        return
    with data_lock:
        gyro_data["x"].append(parsed["gx"])
        gyro_data["y"].append(parsed["gy"])
        gyro_data["z"].append(parsed["gz"])
        accel_data["x"].append(parsed["ax"])
        accel_data["y"].append(parsed["ay"])
        accel_data["z"].append(parsed["az"])


def moving_average(values: deque[float], window: int) -> float:
    if not values:
        return 0.0
    size = min(len(values), window)
    tail = list(values)[-size:]
    return sum(tail) / size


def mock_line() -> str:
    gx, gy, gz = (random.uniform(-2.0, 2.0) for _ in range(3))
    ax, ay, az = (random.uniform(-1.0, 1.0) for _ in range(3))
    return (
        f"Gyro: X |{gx:.4f}|, Y|{gy:.4f}|, Z|{gz:.4f}| --- | "
        f"Accel: X |{ax:.4f}|, Y | {ay:.4f}|, Z |{az:.4f}|"
    )


def serial_worker() -> None:
    if MOCK_DATA:
        while not stop_event.is_set():
            ingest_line(mock_line())
            time.sleep(MOCK_INTERVAL)
        return

    if not ser or not ser.is_open:
        print("Serial reader: Port not open, thread exiting.")
        return

    try:
        while ser and ser.is_open and not stop_event.is_set():
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if line:
                ingest_line(line)
    except Exception as exc:
        print(f"Serial reader stopped: {exc}")


def start_serial() -> None:
    global ser, serial_thread, serial_started
    with serial_lock:
        if serial_started:
            return
        serial_started = True
        stop_event.clear()

        if not MOCK_DATA:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                print(f"Connected to {SERIAL_PORT}")
            except Exception as exc:
                print(f"Could not open port: {exc}")
                ser = None
                serial_started = False
                return

        serial_thread = threading.Thread(target=serial_worker, daemon=True)
        serial_thread.start()


def stop_serial() -> None:
    global ser, serial_started
    stop_event.set()
    with serial_lock:
        if ser and ser.is_open:
            ser.close()
        ser = None
        serial_started = False


def chart_options() -> dict:
    return {
        "backgroundColor": "transparent",
        "tooltip": {"trigger": "axis"},
        "legend": {
            "data": ["X", "Y", "Z"],
            "top": 6,
            "textStyle": {"color": "#9ca3af", "fontSize": 10},
        },
        "grid": {
            "left": 12,
            "right": 12,
            "top": 40,
            "bottom": 24,
            "containLabel": True,
        },
        "xAxis": {
            "type": "category",
            "data": [],
            "axisLine": {"lineStyle": {"color": "rgba(255,255,255,0.15)"}},
            "axisTick": {"show": False},
            "axisLabel": {"color": "#9ca3af", "fontSize": 10},
            "splitLine": {"show": False},
        },
        "yAxis": {
            "type": "value",
            "axisLine": {"lineStyle": {"color": "rgba(255,255,255,0.15)"}},
            "axisTick": {"show": False},
            "axisLabel": {"color": "#9ca3af", "fontSize": 10},
            "splitLine": {"lineStyle": {"color": "rgba(255,255,255,0.08)"}},
        },
        "series": [
            {
                "name": "X",
                "type": "line",
                "data": [],
                "smooth": True,
                "showSymbol": False,
                "lineStyle": {"color": COLOR_X, "width": 2},
            },
            {
                "name": "Y",
                "type": "line",
                "data": [],
                "smooth": True,
                "showSymbol": False,
                "lineStyle": {"color": COLOR_Y, "width": 2},
            },
            {
                "name": "Z",
                "type": "line",
                "data": [],
                "smooth": True,
                "showSymbol": False,
                "lineStyle": {"color": COLOR_Z, "width": 2},
            },
        ],
        "animation": False,
    }


def update_chart(chart, series_data: list[list[float]]) -> None:
    length = len(series_data[0]) if series_data else 0
    chart.options["xAxis"]["data"] = list(range(length))
    for idx, data in enumerate(series_data):
        chart.options["series"][idx]["data"] = data
    chart.update()


def build_ui() -> None:
    ui.dark_mode()
    ui.add_head_html(
        """
<style>
body {
    background: #1a1a1a;
    color: #e5e7eb;
}
.nicegui-content {
    max-width: 1400px;
    margin: 0 auto;
    padding: 24px;
}
.panel {
    background: #2f2f2f;
    border: 1px solid #3d3d3d;
    border-radius: 18px;
    box-shadow: 0 16px 32px rgba(0, 0, 0, 0.35);
}
.panel-title {
    font-size: 12px;
    text-transform: uppercase;
    letter-spacing: 0.12em;
    color: #cbd5f5;
    margin-bottom: 8px;
}
.placeholder {
    background: #4b4b4b;
    border-radius: 16px;
    color: #d1d5db;
    height: 180px;
}
</style>
"""
    )

    with ui.grid(columns=2).classes("w-full gap-6 items-start"):
        # LEFT COLUMN: Graphs
        with ui.column().classes("w-full gap-6"):
            with ui.card().classes("panel w-full"):
                ui.label("Gyroscope Data").classes("panel-title")
                gyro_chart = (
                    ui.echart(chart_options()).classes("w-full").style("height: 400px;")
                )
            with ui.card().classes("panel w-full"):
                ui.label("Accelerometer Data").classes("panel-title")
                accel_chart = (
                    ui.echart(chart_options()).classes("w-full").style("height: 400px;")
                )

        # RIGHT COLUMN: 3D Model & Video Placeholder
        with ui.column().classes("w-full gap-6"):
            with ui.card().classes("panel w-full"):
                ui.label("3D STL Model").classes("panel-title")
                with ui.scene(height=400, grid=False, background_color="#0a0a0a").classes(
                    "w-full"
                ) as scene:
                    # Load the model. Adjust key/scale as needed
                    scene.stl("/stls/AnthonyHua.stl").material("#ff9100").scale(0.05)
                    scene.cylinder(0.2, 0.2, 2).move(x=0.6).rotate(0, 0, 1.5).material(
                        "#ff4444"
                    ).scale(
                        0.55
                    )  # X-axis marker
                    scene.cylinder(0.2, 0.2, 2).move(y=0.6).material("#44ff44").scale(
                        0.55
                    )  # Y-axis marker
                    scene.cylinder(0.2, 0.2, 2).move(z=0.6).rotate(1.5, 0, 0).material(
                        "#4444ff"
                    ).scale(
                        0.55
                    )  # Z-axis marker
                    scene.move_camera(0, -10, 5)

            # Video Placeholder
            with ui.card().classes("panel w-full"):
                ui.label("Video Feed (Placeholder)").classes("panel-title")
                with ui.element("div").classes(
                    "placeholder w-full h-64 flex items-center justify-center"
                ):
                    ui.label("Video Feed Area")

    def update_ui() -> None:
        with data_lock:
            gx = list(gyro_data["x"])
            gy = list(gyro_data["y"])
            gz = list(gyro_data["z"])
            ax = list(accel_data["x"])
            ay = list(accel_data["y"])
            az = list(accel_data["z"])

        update_chart(gyro_chart, [gx, gy, gz])
        update_chart(accel_chart, [ax, ay, az])

    ui.timer(UPDATE_INTERVAL, update_ui)


app.on_startup(start_serial)
app.on_shutdown(stop_serial)
ui.run(build_ui, title="MPU6500 Sensor Dashboard", reload=False)
