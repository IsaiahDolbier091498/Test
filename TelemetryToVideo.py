import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from math import radians, sin, cos

# Loads in Telemetry file
telemetryData = pd.read_csv('Telemetry.csv')

frames = len(telemetryData)

# Turns each column of the csv file into separate lists
altitudeData = telemetryData['Altitude (m)']
velocityData = telemetryData['Velocity (m/s)']
rollData = telemetryData['Roll (deg)']
pitchData = telemetryData['Pitch (deg)']
yawData = telemetryData['Yaw (deg)']
servo1Data = telemetryData['Servo1 (deg)']
servo2Data = telemetryData['Servo2 (deg)']
servo3Data = telemetryData['Servo3 (deg)']
servo4Data = telemetryData['Servo4 (deg)']
timestampData = telemetryData['Timestamp (milliseconds)']

# 3D setup
fig = plt.figure(figsize=(15.5, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-0.2, 0.2])
ax.set_ylim([-0.2, 0.2])
ax.set_zlim([0, 3])
ax.set_box_aspect([0.2, 0.2, 2])
ax.axis('off')
fig.patch.set_facecolor('#F0F0F0')
ax.set_facecolor('#F0F0F0')
ax.view_init(azim=-60, )

translateX = -0.5
translateY = 0
translateZ = 0

# Telemetry text placeholders
altitudeText = fig.text(0.80, 0.9, f'Altitude: {altitudeData[0]:.3f}', fontsize=20, color='black')
velocityText = fig.text(0.80, 0.81, f'Velocity: {velocityData[0]:.3f}', fontsize=20, color='black')
rollText = fig.text(0.80, 0.72, f'Roll: {rollData[0]:.3f}', fontsize=20, color='black')
pitchText = fig.text(0.80, 0.63, f'Pitch: {pitchData[0]:.3f}', fontsize=20, color='black')
yawText = fig.text(0.80, 0.54, f'Yaw: {yawData[0]:.3f}', fontsize=20, color='black')
servo1Text = fig.text(0.80, 0.45, f'Servo1: {servo1Data[0]:.3f}', fontsize=20, color='black')
servo2Text = fig.text(0.80, 0.36, f'Servo2: {servo2Data[0]:.3f}', fontsize=20, color='black')
servo3Text = fig.text(0.80, 0.27, f'Servo3: {servo3Data[0]:.3f}', fontsize=20, color='black')
servo4Text = fig.text(0.80, 0.18, f'Servo4: {servo4Data[0]:.3f}', fontsize=20, color='black')
timestampText = fig.text(0.02, 0.95, f'Time: {timestampData[0]:.3f}', fontsize=20, color='black')

# Returns coordinates to construct the rocket's body
def createCylinder(r=0.15, h=2.0, res=30):
    t = np.linspace(0, 2*np.pi, res)
    z = np.linspace(0, h, 2)
    T, Z = np.meshgrid(t, z)
    return r*np.cos(T), r*np.sin(T), Z

# returns coordinates to construct the rocket's nosecone (ogive shaped)
def createOgive(radius=0.15, height=0.6, res=30):
    zp = np.linspace(0, height, res)
    R0 = (radius**2 + height**2)/(2*radius)
    xp = np.sqrt(R0**2 - (height-zp)**2) - (R0-radius)
    theta, Z = np.meshgrid(np.linspace(0,2*np.pi,res), zp)
    X = xp[:,None]*np.cos(theta)
    Y = xp[:,None]*np.sin(theta)
    return X, Y, height+2.0 - Z

# returns coordinates to construct the rocket's bottom fins
def createBottomFins(bw=0.24, tw=0.24, h=0.35):
    x = np.array([[0,0],[bw,tw]]) - 0.055
    y = np.zeros_like(x)
    z = np.array([[0,h],[0.1,h-0.1]])
    return x, y, z

# returns coordinates to construct the rocket's top fins
def createTopFins(bw=0.14, tw=0.14, h=0.25):
    x = np.array([[0,0],[bw,tw]]) - 0.055
    y = np.zeros_like(x)
    z = np.array([[0,h],[0.1,h-0.1]]) + 1.85
    return x, y, z

# Rotates 3d points given pitch (x-axis), roll (y-axis), and yaw (z-axis)
def rotatePoints(x, y, z, pitch, roll, yaw):
    pts = np.vstack((x.ravel(), y.ravel(), z.ravel())).T
    rot = R.from_euler('zyx', [roll, pitch, yaw], degrees=True)
    new = rot.apply(pts)
    return (new[:,0].reshape(x.shape),
            new[:,1].reshape(y.shape),
            new[:,2].reshape(z.shape))

# Generate the 3D coordinates for the rocket's components
bodyX, bodyY, bodyZ = createCylinder()
ogiveX, ogiveY, ogiveZ = createOgive()
bFx, bFy, bFz = createBottomFins()
tFx, tFy, tFz = createTopFins()

# Placeholder surfaces for each body
bodySurf = None
ogiveSurf = None
bFinSurfs = []
tFinSurfs = []

# Updates every frame
def update(i):
    altitudeText.set_text(f'Altitude: {altitudeData[i]:.3f} (m)')
    velocityText.set_text(f'Velocity: {velocityData[i]:.3f} (m/s)')
    rollText.set_text(f'Roll: {rollData[i]:.3f} (°)')
    pitchText.set_text(f'Pitch: {pitchData[i]:.3f} (°)')
    yawText.set_text(f'Yaw: {yawData[i]:.3f} (°)')
    servo1Text.set_text(f'Servo1: {79 - servo1Data[i]:.3f} (°)')
    servo2Text.set_text(f'Servo2: {98 - servo2Data[i]:.3f} (°)')
    servo3Text.set_text(f'Servo3: {88 - servo3Data[i]:.3f} (°)')
    servo4Text.set_text(f'Servo4: {86 - servo4Data[i]:.3f} (°)')
    timestampText.set_text(f'Time: {timestampData[i]:.3f} (ms) {timestampData[i]//1000} (s)')

    global bodySurf, ogiveSurf, bFinSurfs, tFinSurfs

    if bodySurf: bodySurf.remove()
    if ogiveSurf: ogiveSurf.remove()
    for s in bFinSurfs: s.remove()
    for s in tFinSurfs: s.remove()
    bFinSurfs.clear()
    tFinSurfs.clear()

    p, r, yw = pitchData[i], rollData[i], yawData[i]

    bx, by, bz = rotatePoints(bodyX, bodyY, bodyZ, p, r, yw)
    bx += translateX
    by += translateY
    bz += translateZ
    bodySurf = ax.plot_surface(bx, by, bz, color='yellow', alpha=0.6)

    ox, oy, oz = rotatePoints(ogiveX, ogiveY, ogiveZ, p, r, yw)
    ox += translateX
    oy += translateY
    oz += translateZ
    ogiveSurf = ax.plot_surface(ox, oy, oz, color='black', alpha=0.8)

    for ang in [45,135,225,315]:
        rad = radians(ang)
        x = bFx.copy(); y = bFy.copy(); z = bFz.copy()
        x2 = x*cos(rad) - y*sin(rad)
        y2 = x*sin(rad) + y*cos(rad)
        x2 += (0.205)*cos(rad); y2 += (0.205)*sin(rad)
        fx, fy, fz = rotatePoints(x2, y2, z, p, r, yw)
        bFinSurfs.append(ax.plot_surface(fx + translateX, fy + translateY, fz + translateZ, color='black', zorder=5))

    servos = [79 - servo1Data[i],98 - servo2Data[i],88 - servo3Data[i],86 - servo4Data[i]]
    for baseAng, servoAng in zip([0,90,180,270], servos):
        pts = np.vstack((tFx.ravel(), tFy.ravel(), tFz.ravel())).T
        pivot = pts[[2, 3], :].mean(axis=0)
        pts -= pivot
        pts = R.from_euler('x', servoAng, degrees=True).apply(pts)
        pts += pivot
        pts = R.from_euler('z', baseAng, degrees=True).apply(pts)
        x2 = pts[:,0].reshape(tFx.shape)
        y2 = pts[:,1].reshape(tFy.shape)
        z2 = pts[:,2].reshape(tFz.shape)
        off = 0.205; rad = radians(baseAng)
        x2 += off*cos(rad); y2 += off*sin(rad)
        fx, fy, fz = rotatePoints(x2, y2, z2, p, r, yw)
        tFinSurfs.append(ax.plot_surface(fx + translateX, fy + translateY, fz + translateZ, color='red', zorder=5))

# Writer setup

avgTimestamp = np.mean(np.diff(timestampData))
avgFps = 1000 / avgTimestamp
print(avgFps)

writer = FFMpegWriter(fps=avgFps)  # Adjust fps to match the real-time rate based on telemetry update intervals

# Animation
ani = FuncAnimation(fig, update, frames=frames, interval=1, blit=False)

# Save the animation
ani.save('flight_animation.mp4', writer=writer, dpi=200)
