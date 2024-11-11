import serial
import time
import matplotlib.pyplot as plt

# Connect to the Arduino's serial port
ser = serial.Serial('COM4', 115200)  # Update 'COM3' to match the Arduino port

# Initialize data lists
times = []
positions = []

# Set up the plot
plt.ion()  # Interactive mode on
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')  # Initialize an empty line for updating
ax.set_xlim(0, 20)             # Set x-axis to 20 seconds
ax.set_ylim(-1000, 1000)       # Adjust y-axis based on expected encoder range

start_time = time.time()

# Read and plot data for 20 seconds
while time.time() - start_time < 20:
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()  # Read and decode serial data
        try:
            t, position = map(float, data.split(","))
            times.append(t)
            positions.append(position)

            # Update the plot
            line.set_xdata(times)
            line.set_ydata(positions)
            ax.relim()
            ax.autoscale_view()

            plt.draw()
            plt.pause(0.01)  # Pause to allow plot update

        except ValueError:
            print("Data format error, skipping:", data)
            continue

# Close serial and finalize plot
ser.close()
plt.ioff()
plt.show()  # Keep final plot open
