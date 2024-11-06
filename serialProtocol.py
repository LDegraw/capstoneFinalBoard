import serial
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from collections import deque
import matplotlib.animation as animation

class LiveAccelerationPlotter:
    def __init__(self, serial_port='COM3', baudrate=115200, max_points=20):  # Reduced max points
        # Initialize serial connection
        self.ser = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"Connected to {self.ser.name}")

        # Initialize figure and axes
        self.fig = plt.figure(figsize=(15, 6))
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1, 2])
        
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1], projection='3d')
        
        # Create line objects
        self.line_x, = self.ax1.plot([], [], label='X', color='#FF6B6B')
        self.line_y, = self.ax1.plot([], [], label='Y', color='#4ECDC4')
        self.line_z, = self.ax1.plot([], [], label='Z', color='#45B7D1')
        
        # Create 3D line for position tracking
        self.line_3d, = self.ax2.plot3D([], [], [], color='blue', linewidth=2)
        
        # Initialize data storage with smaller buffer
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.x_data = deque(maxlen=max_points)
        self.y_data = deque(maxlen=max_points)
        self.z_data = deque(maxlen=max_points)
        
        # Position and velocity storage
        self.pos_x = deque(maxlen=max_points)
        self.pos_y = deque(maxlen=max_points)
        self.pos_z = deque(maxlen=max_points)
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.last_time = None
        
        # Filter parameters
        self.alpha = 0.1
        self.filtered_x = 0
        self.filtered_y = 0
        self.filtered_z = 0
        
        # Calibration
        self.calibration_samples_x = []
        self.calibration_samples_y = []
        self.calibration_samples_z = []
        self.calibration_count = 0
        self.CALIBRATION_NEEDED = 20
        self.is_calibrated = False
        self.x_offset = 0
        self.y_offset = 0
        self.gravity_scale = None
        
        # 3D plot parameters
        self.view_margin = 0.5
        self.min_view_size = 0.5
        
        # Setup axes
        self._setup_axes()
        print("Starting calibration... Keep the device still...")

    def _apply_low_pass_filter(self, new_value, filtered_value):
        return self.alpha * new_value + (1 - self.alpha) * filtered_value

    def _update_3d_view(self):
        if len(self.pos_x) > 0:
            x, y, z = list(self.pos_x), list(self.pos_y), list(self.pos_z)
            
            # Calculate ranges using only recent points
            x_range = max(x) - min(x)
            y_range = max(y) - min(y)
            z_range = max(z) - min(z)
            
            max_range = max(max(x_range, y_range, z_range), self.min_view_size)
            view_size = max_range + self.view_margin
            
            x_center = (max(x) + min(x)) / 2
            y_center = (max(y) + min(y)) / 2
            z_center = (max(z) + min(z)) / 2
            
            self.ax2.set_xlim3d(x_center - view_size, x_center + view_size)
            self.ax2.set_ylim3d(y_center - view_size, y_center + view_size)
            self.ax2.set_zlim3d(z_center - view_size, z_center + view_size)
            
            # Show current position and velocity
            self.ax2.set_title(f'Position (m) | Velocity (m/s):\nX: {x[-1]:.2f} | {self.vel_x:.2f}\nY: {y[-1]:.2f} | {self.vel_y:.2f}\nZ: {z[-1]:.2f} | {self.vel_z:.2f}')
        
    def _calibrate_sensors(self, x, y, z):
        """Calibrate all axes"""
        if self.calibration_count < self.CALIBRATION_NEEDED:
            self.calibration_samples_x.append(x)
            self.calibration_samples_y.append(y)
            self.calibration_samples_z.append(abs(z))
            self.calibration_count += 1
            print(f"Calibrating... {self.calibration_count}/{self.CALIBRATION_NEEDED}")
            return False
        elif not self.is_calibrated:
            # Calculate offsets for X and Y
            self.x_offset = -np.mean(self.calibration_samples_x)
            self.y_offset = -np.mean(self.calibration_samples_y)
            
            # Calculate gravity scale from Z
            avg_z = np.mean(self.calibration_samples_z)
            self.gravity_scale = 9.81 / avg_z
            
            print(f"Calibration complete:")
            print(f"X offset: {self.x_offset:.3f}")
            print(f"Y offset: {self.y_offset:.3f}")
            print(f"Gravity scale: {self.gravity_scale:.3f}")
            
            self.is_calibrated = True
            return True
        return True

    def _normalize_acceleration(self, x, y, z):
        """Apply calibration to acceleration values"""
        if not self.is_calibrated:
            return x, y, z
        
        x_cal = (x + self.x_offset) * self.gravity_scale
        y_cal = (y + self.y_offset) * self.gravity_scale
        z_cal = z * self.gravity_scale
        
        return x_cal, y_cal, z_cal

    def _parse_value(self, value_str):
        try:
            parts = value_str.split(': ')
            if len(parts) != 2:
                return None
            
            value = parts[1]
            if value.startswith('0.-'):
                value = value.replace('0.-', '-')
            
            return float(value)
        except (IndexError, ValueError) as e:
            print(f"Parse error for {value_str}: {str(e)}")
            return None
        
    def _setup_axes(self):
        self.ax1.set_title('Real-time Acceleration')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Acceleration (m/s²)')
        self.ax1.grid(True)
        self.ax1.legend()
        
        self.ax2.set_title('3D Position')
        self.ax2.set_xlabel('X Position (m)')
        self.ax2.set_ylabel('Y Position (m)')
        self.ax2.set_zlabel('Z Position (m)')
        
        self.ax2.set_box_aspect([2,2,2])
        plt.tight_layout()

    def _update_position(self, ax, ay, az, current_time):
        if self.last_time is None:
            self.last_time = current_time
            self.pos_x.append(0)
            self.pos_y.append(0)
            self.pos_z.append(0)
            return
        
        dt = current_time - self.last_time
        
        # Apply low-pass filter to acceleration values
        self.filtered_x = self._apply_low_pass_filter(ax, self.filtered_x)
        self.filtered_y = self._apply_low_pass_filter(ay, self.filtered_y)
        self.filtered_z = self._apply_low_pass_filter(az, self.filtered_z)
        
        # Update velocities using filtered acceleration
        self.vel_x += self.filtered_x * dt
        self.vel_y += self.filtered_y * dt
        self.vel_z += (self.filtered_z + 9.81) * dt
        
        # Apply velocity damping
        damping = 0.99
        self.vel_x *= damping
        self.vel_y *= damping
        self.vel_z *= damping
        
        # Update positions
        if len(self.pos_x) > 0:
            new_x = self.pos_x[-1] + self.vel_x * dt
            new_y = self.pos_y[-1] + self.vel_y * dt
            new_z = self.pos_z[-1] + self.vel_z * dt
        else:
            new_x = self.vel_x * dt
            new_y = self.vel_y * dt
            new_z = self.vel_z * dt
        
        self.pos_x.append(new_x)
        self.pos_y.append(new_y)
        self.pos_z.append(new_z)
        
        self.last_time = current_time

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline()
                decoded_line = line.decode('utf-8').strip()
                decoded_line = decoded_line.replace('\x00', '')
                values = decoded_line.split(', ')
                
                if len(values) >= 3:
                    x = self._parse_value(values[0])
                    y = self._parse_value(values[1])
                    z = self._parse_value(values[2])
                    
                    if all(v is not None for v in [x, y, z]):
                        return x, y, z
                    else:
                        print(f"Failed to parse one or more values: {values}")
                        return None
            except UnicodeDecodeError:
                print(f"Decode error: {line.hex()}")
                return None
        return None
        
    def update(self, frame):
        data = self.read_serial_data()
        if data is not None:
            x, y, z = data
            current_time = time.time() - self.start_time
            
            if not self._calibrate_sensors(x, y, z):
                return self.line_x, self.line_y, self.line_z, self.line_3d
            
            x, y, z = self._normalize_acceleration(x, y, z)
            
            self.times.append(current_time)
            self.x_data.append(x)
            self.y_data.append(y)
            self.z_data.append(z)
            
            self._update_position(x, y, z, current_time)
            
            self.line_x.set_data(list(self.times), list(self.x_data))
            self.line_y.set_data(list(self.times), list(self.y_data))
            self.line_z.set_data(list(self.times), list(self.z_data))
            
            self.line_3d.set_data_3d(list(self.pos_x), list(self.pos_y), list(self.pos_z))
            
            self.ax1.relim()
            self.ax1.autoscale_view()
            
            self._update_3d_view()
            
        return self.line_x, self.line_y, self.line_z, self.line_3d
        
    def start(self, interval=50):
        self.start_time = time.time()
        self.ani = animation.FuncAnimation(
            self.fig, 
            self.update, 
            interval=interval, 
            blit=True,
            cache_frame_data=False,
            save_count=100
        )
        plt.show()

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    try:
        plotter = LiveAccelerationPlotter(serial_port='COM3', baudrate=115200)
        plotter.start()
    except serial.SerialException as e:
        print(f"Error: Could not open COM3 port. {str(e)}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'plotter' in locals():
            plotter.close()