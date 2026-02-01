import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# --- CONFIGURATION ---
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
MAX_POINTS = 100 

# --- DATA BUFFERS ---
gyro_x = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
gyro_y = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
gyro_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

accel_x = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
accel_y = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
accel_z = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

x_axis = deque(range(MAX_POINTS), maxlen=MAX_POINTS)

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
    parts = line.split('|')
    
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
    while ser.in_waiting > 0:
        try:
            raw_line = ser.readline()
            line = raw_line.decode('utf-8', errors='ignore').strip()
            
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

    # -- Draw Gyro --
    ax1.clear()
    ax1.plot(gyro_x, label='X', color='red')
    ax1.plot(gyro_y, label='Y', color='green')
    ax1.plot(gyro_z, label='Z', color='blue')
    ax1.set_title("Gyroscope Data")
    ax1.legend(loc='upper right', fontsize='x-small')
    ax1.grid(True, linestyle=':', alpha=0.6)

    # -- Draw Accel --
    ax2.clear()
    ax2.plot(accel_x, label='X', color='red')
    ax2.plot(accel_y, label='Y', color='green')
    ax2.plot(accel_z, label='Z', color='blue')
    ax2.set_title("Accelerometer Data")
    ax2.legend(loc='upper right', fontsize='x-small')
    ax2.grid(True, linestyle=':', alpha=0.6)

# --- PLOT SETUP ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
plt.subplots_adjust(hspace=0.3) 

ani = animation.FuncAnimation(fig, update_graph, interval=50)
plt.show()

ser.close()