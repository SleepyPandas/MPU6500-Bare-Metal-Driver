import serial
import threading
from nicegui import ui, app

# Setup Serial (Global)
ser = None
try:
    # This replaces PuTTY
    ser = serial.Serial("COM3", 115200, timeout=1)
except Exception as e:
    print(f"Could not open port: {e}")


def read_serial_loop():
    """This function runs in the background forever, just like PuTTY."""
    while ser and ser.is_open:
        if ser.in_waiting > 0:
            try:
                # Read the line
                line = ser.readline().decode("utf-8").strip()

                # 1. Print to VS Code terminal (So you can see it like PuTTY)
                print(f"RAW: {line}")

                # 2. Update NiceGUI
                log_element.push(line)
            except:
                pass


# Create a UI Log
log_element = ui.log().classes("w-full h-40")

# Start the reader thread when the app starts
if ser:
    threading.Thread(target=read_serial_loop, daemon=True).start()

# Clean up on close
app.on_shutdown(lambda: ser.close() if ser else None)

ui.run()
