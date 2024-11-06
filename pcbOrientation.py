import serial
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import keyboard  

class PCBOrientationVisualizer:
    def __init__(self):
        # Create figure and 3D axes
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # PCB board vertices (rectangular shape, thinner in Z axis)
        self.vertices = np.array([
            [-1.0, -1.5, -0.1],  # Bottom rectangle
            [1.0, -1.5, -0.1],
            [1.0, 1.5, -0.1],
            [-1.0, 1.5, -0.1],
            [-1.0, -1.5, 0.1],   # Top rectangle
            [1.0, -1.5, 0.1],
            [1.0, 1.5, 0.1],
            [-1.0, 1.5, 0.1]
        ])
        
        # Define board faces
        self.faces = [
            [self.vertices[j] for j in [0, 1, 2, 3]],  # Bottom
            [self.vertices[j] for j in [4, 5, 6, 7]],  # Top
            [self.vertices[j] for j in [0, 1, 5, 4]],  # Front
            [self.vertices[j] for j in [2, 3, 7, 6]],  # Back
            [self.vertices[j] for j in [0, 3, 7, 4]],  # Left
            [self.vertices[j] for j in [1, 2, 6, 5]]   # Right
        ]
        
        # Colors for each face
        self.face_colors = ['darkgreen', 'darkgreen', 'green', 'green', 'green', 'green']
        
        # Calibration offsets (initialized to 0)
        self.pitch_offset = 0
        self.roll_offset = 0
        
        # Current angles
        self.current_pitch = 0
        self.current_roll = 0
        
        # Initial plot setup
        self.setup_plot()
        
        # Setup keyboard listener for spacebar
        keyboard.on_press_key('space', self.handle_spacebar)

    def handle_spacebar(self, _):
        """Handle spacebar press to calibrate offsets"""
        # Add current position to existing offsets to zero the display
        self.pitch_offset += self.current_pitch
        self.roll_offset += self.current_roll
        print(f"\n[CALIBRATED] Offsets updated - Pitch: {self.pitch_offset}°, Roll: {self.roll_offset}°")
        self.visualize(self.current_pitch, self.current_roll)

    def setup_plot(self):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_title('PCB Orientation Visualization\nPress SPACEBAR to set current position to zero')

    def get_calibrated_angles(self, pitch, roll):
        """Calculate calibrated angles by subtracting offsets"""
        cal_pitch = pitch - self.pitch_offset
        cal_roll = roll - self.roll_offset
        return cal_pitch, cal_roll

    def rotation_matrix(self, pitch, roll):
        """Create rotation matrix from pitch and roll angles (in degrees)"""
        # Convert to radians
        pitch = np.radians(pitch)
        roll = np.radians(roll)
        
        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])
        
        Ry = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])
        
        # Combined rotation
        return np.dot(Ry, Rx)

    def visualize(self, pitch, roll):
        """Update visualization with given pitch and roll values"""
        self.current_pitch = pitch
        self.current_roll = roll
        
        self.ax.cla()
        self.setup_plot()
        
        # Get calibrated angles
        cal_pitch, cal_roll = self.get_calibrated_angles(pitch, roll)
        
        # Apply rotation using calibrated angles
        R = self.rotation_matrix(cal_pitch, cal_roll)
        rotated_vertices = np.dot(self.vertices, R.T)
        
        # Update faces with rotated vertices
        rotated_faces = [[rotated_vertices[j] for j in [0, 1, 2, 3]],
                        [rotated_vertices[j] for j in [4, 5, 6, 7]],
                        [rotated_vertices[j] for j in [0, 1, 5, 4]],
                        [rotated_vertices[j] for j in [2, 3, 7, 6]],
                        [rotated_vertices[j] for j in [0, 3, 7, 4]],
                        [rotated_vertices[j] for j in [1, 2, 6, 5]]]
        
        # Plot the rotated PCB board
        collection = Poly3DCollection(rotated_faces, facecolors=self.face_colors, alpha=0.95)
        collection.set_edgecolor('black')
        self.ax.add_collection3d(collection)
        
        # Add text to show angles
        raw_text = f'Raw Angles:\nPitch: {pitch:.1f}°\nRoll: {roll:.1f}°'
        cal_text = f'Calibrated Angles:\nPitch: {cal_pitch:.1f}°\nRoll: {cal_roll:.1f}°'
        offset_text = f'Offsets:\nPitch: {self.pitch_offset:.1f}°\nRoll: {self.roll_offset:.1f}°'
        
        self.ax.text2D(0.05, 0.95, raw_text, transform=self.ax.transAxes)
        self.ax.text2D(0.65, 0.95, cal_text, transform=self.ax.transAxes)
        self.ax.text2D(0.35, 0.95, offset_text, transform=self.ax.transAxes)
        
        plt.draw()
        plt.pause(0.01)
    def calibrate(self, pitch, roll):
        """Set calibration offsets based on current position"""
        self.pitch_offset = pitch
        self.roll_offset = roll
Port = serial.Serial('COM3', baudrate=115200, timeout=1)
pitchEstimate = 0
firstGBool = 1
firstGVal = 0
dataDictionary = {
    "AvgX" : 0,
    "AvgY" : 0,
    "AvgZ" : 0
}

plot = PCBOrientationVisualizer()
calibrated = False

while(True):
    if(Port.in_waiting):
        line = Port.readline()
        decoded_line = line.decode('utf-8').strip()
        decoded_line = decoded_line.replace('\x00', '')
        values = decoded_line.split(', ')
        if(len(values) > 1):
            for value in values:
                valuePair = value.split(': ')
                try:
                    dataDictionary[valuePair[0]] = float(valuePair[1])

                except:
                    print("Wierd")
        if(firstGBool):
            if(dataDictionary['AvgZ'] > 0):
                print("Calibration Value")
                firstGVal = dataDictionary['AvgZ']
                firstGBool = 0
                print(firstGVal)
        try:
            rollEstimate = math.degrees(math.atan(dataDictionary['AvgY']/dataDictionary['AvgZ']))
            print(f"Roll Angle: {rollEstimate}")

        except:
            print("domain or something")
        try:
            pitchEstimate = math.degrees(math.asin(dataDictionary['AvgX']/firstGVal))
            print(f"pitchEstimate: {pitchEstimate}")
        
        except:
            print("domain or something")
        
        
        if(not calibrated):
            while True:
                # Get user input
                print("\n'c' to calibrate current position as zero):")
                user_input = input("ENTER: ")

                # Check for calibration command
                if user_input.lower() == 'c':
                    plot.calibrate(pitchEstimate, rollEstimate)
                    calibrated = True
                    break
        time.sleep(0.01)
        plot.visualize(pitchEstimate, rollEstimate)
    