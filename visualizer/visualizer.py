from vpython import box, vector, rate
import serial
import re
from math import radians, cos, sin

# Connect to your Teensy via Serial (adjust COM port to yours)
ser = serial.Serial('COM5', 115200)  # Replace 'COM5' with your Teensy's actual port
rocket = box(length=2, height=0.2, width=0.2, color=vector(1, 0, 0))

# Function to parse the data coming from Teensy
def parse_line(line):
    match = re.search(r'Pitch:\s*(-?\d+\.\d+)\s*\|\s*Roll:\s*(-?\d+\.\d+)\s*\|\s*Yaw:\s*(-?\d+\.\d+)', line)
    if match:
        return float(match.group(1)), float(match.group(2)), float(match.group(3))
    return None

# Main loop for reading serial data and visualizing the rocket
while True:
    rate(30)  # Adjust this to control update speed for visualization
    if ser.in_waiting:  # If there is data available from serial
        line = ser.readline().decode('utf-8').strip()
        print("RAW:", line)  # Debug print, check if the data is coming in
        data = parse_line(lie)  # Parse the pitch, roll, and yaw from the line
        if data:
            pitch, roll, yaw = map(radians, data)  # Convert degrees to radians
            print(f"Updating rocket orientation - Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
            rocket.axis = vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))  # Update rocket orientation
            rocket.up = vector(-sin(roll), cos(roll), 0)  # Update rocket 'up' vector
            print(f"Rocket orientation updated: Axis: {rocket.axis}, Up: {rocket.up}")  # Debug print for orientation
            