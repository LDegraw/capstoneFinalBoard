import tkinter as tk
from tkinter import ttk, messagebox
import cv2
import serial
import glob
import threading
from PIL import Image, ImageTk
import time

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Interface")
        self.root.geometry("1200x800")
        
        # Initialize variables
        self.serial = None
        self.is_connected = False
        self.is_pump_active = False
        self.is_camera_active = False
        self.camera = None
        self.camera_thread = None
        self.camera_lock = threading.Lock()
        
        # Initialize sensor variables
        self.avg_x = tk.StringVar(value="0.00")
        self.avg_y = tk.StringVar(value="0.00")
        self.avg_z = tk.StringVar(value="0.00")
        self.force_f = tk.StringVar(value="0.00")
        self.force_l = tk.StringVar(value="0.00")
        self.force_r = tk.StringVar(value="0.00")
        self.force_b = tk.StringVar(value="0.00")
        
        # Initialize serial reading thread
        self.serial_thread = None
        self.is_reading = False
        
        # Initialize status variable before creating layout
        self.status_var = tk.StringVar(value="System Ready - Not Connected")
        
        # Create main frames
        self.create_layout()
        
        # Initialize controls
        self.setup_controls()
        
        # Key bindings
        self.setup_key_bindings()
        
    def create_layout(self):
        # Original layout code remains the same until control_frame
        self.main_container = ttk.Frame(self.root, padding="10")
        self.main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Left panel for camera
        self.camera_frame = ttk.LabelFrame(self.main_container, text="Camera Feed", padding="5")
        self.camera_frame.grid(row=0, column=0, rowspan=2, padx=5, pady=5, sticky="nsew")
        
        # Camera display
        self.camera_label = ttk.Label(self.camera_frame)
        self.camera_label.grid(row=0, column=0, padx=5, pady=5)
        
        # Right panel for controls and sensor data
        self.right_panel = ttk.Frame(self.main_container)
        self.right_panel.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        
        # Control frame
        self.control_frame = ttk.Frame(self.right_panel)
        self.control_frame.grid(row=0, column=0, sticky="nsew")
        
        # Sensor data frame
        self.sensor_frame = ttk.LabelFrame(self.right_panel, text="Sensor Data", padding="5")
        self.sensor_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        # Setup sensor display
        self.setup_sensor_display()
        
        # Status bar
        self.status_bar = ttk.Label(self.main_container, textvariable=self.status_var, relief="sunken")
        self.status_bar.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(5,0))
        
        # Configure grid weights
        self.main_container.grid_columnconfigure(0, weight=3)
        self.main_container.grid_columnconfigure(1, weight=1)

    def setup_sensor_display(self):
        # Average Values
        avg_frame = ttk.LabelFrame(self.sensor_frame, text="Average Position", padding="5")
        avg_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        ttk.Label(avg_frame, text="X:").grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(avg_frame, textvariable=self.avg_x).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(avg_frame, text="Y:").grid(row=1, column=0, padx=5, pady=2)
        ttk.Label(avg_frame, textvariable=self.avg_y).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(avg_frame, text="Z:").grid(row=2, column=0, padx=5, pady=2)
        ttk.Label(avg_frame, textvariable=self.avg_z).grid(row=2, column=1, padx=5, pady=2)
        
        # Force Values
        force_frame = ttk.LabelFrame(self.sensor_frame, text="Force Readings", padding="5")
        force_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        ttk.Label(force_frame, text="Forward:").grid(row=0, column=0, padx=5, pady=2)
        ttk.Label(force_frame, textvariable=self.force_f).grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(force_frame, text="Left:").grid(row=1, column=0, padx=5, pady=2)
        ttk.Label(force_frame, textvariable=self.force_l).grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(force_frame, text="Right:").grid(row=2, column=0, padx=5, pady=2)
        ttk.Label(force_frame, textvariable=self.force_r).grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(force_frame, text="Back:").grid(row=3, column=0, padx=5, pady=2)
        ttk.Label(force_frame, textvariable=self.force_b).grid(row=3, column=1, padx=5, pady=2)

    def start_serial_reading(self):
        """Start the serial reading thread"""
        self.is_reading = True
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def stop_serial_reading(self):
        """Stop the serial reading thread"""
        self.is_reading = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)

    def read_serial_data(self):
        """Read and parse serial data"""
        while self.is_reading and self.serial:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    self.parse_sensor_data(line)
            except Exception as e:
                print(f"Error reading serial: {e}")
                break
            time.sleep(0.01)

    def parse_sensor_data(self, data):
        """Parse the incoming sensor data and update GUI"""
        try:
            # Split the data string and extract values
            parts = data.split(', ')
            for part in parts:
                if ':' in part:
                    key, value = part.split(': ')
                    value = float(value)
                    
                    # Update corresponding variable
                    if key == 'AvgX':
                        self.avg_x.set(f"{value:.2f}")
                    elif key == 'AvgY':
                        self.avg_y.set(f"{value:.2f}")
                    elif key == 'AvgZ':
                        self.avg_z.set(f"{value:.2f}")
                    elif key == 'F':
                        self.force_f.set(f"{value:.2f}")
                    elif key == 'L':
                        self.force_l.set(f"{value:.2f}")
                    elif key == 'R':
                        self.force_r.set(f"{value:.2f}")
                    elif key == 'B':
                        self.force_b.set(f"{value:.2f}")
        except Exception as e:
            print(f"Error parsing data: {e}")

    def toggle_connection(self):
        """Connect or disconnect from the selected serial port"""
        if not self.is_connected:
            port = self.port_var.get()
            try:
                self.serial = serial.Serial(port, 115200, timeout=1)
                self.is_connected = True
                self.connect_btn.configure(text="Disconnect")
                self.status_var.set(f"Connected to {port}")
                self.port_combo.configure(state="disabled")
                self.start_serial_reading()  # Start reading serial data
            except serial.SerialException as e:
                messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
                self.status_var.set("Connection failed")
        else:
            self.stop_serial_reading()  # Stop reading serial data
            if self.serial:
                self.serial.close()
            self.is_connected = False
            self.connect_btn.configure(text="Connect")
            self.port_combo.configure(state="normal")
            self.status_var.set("Disconnected")

    def cleanup(self):
        """Cleanup resources before closing"""
        self.stop_serial_reading()  # Stop serial reading thread
        if self.is_camera_active:
            self.stop_camera()
        if self.serial:
            self.serial.close()
        self.root.destroy()

    def setup_controls(self):
        # Connection Controls
        connection_frame = ttk.LabelFrame(self.control_frame, text="Connection Settings", padding="5")
        connection_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        # Port selection
        ttk.Label(connection_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        # Refresh and connect buttons
        ttk.Button(connection_frame, text="Refresh Ports", command=self.refresh_ports).grid(row=1, column=0, padx=5, pady=5)
        self.connect_btn = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=1, padx=5, pady=5)
        
        # Initial port refresh
        self.refresh_ports()
        
        # Camera Controls
        camera_control = ttk.LabelFrame(self.control_frame, text="Camera Control", padding="5")
        camera_control.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        self.camera_btn = ttk.Button(camera_control, text="Start Camera",
                                   command=self.toggle_camera)
        self.camera_btn.grid(row=0, column=0, padx=5, pady=5)
        
        # Motor Controls
        motor_control = ttk.LabelFrame(self.control_frame, text="Base Motor Control", padding="5")
        motor_control.grid(row=2, column=0, padx=5, pady=5, sticky="ew")
        
        # Motor direction buttons
        ttk.Button(motor_control, text="↑", command=lambda: self.send_motor_command('U')).grid(row=0, column=1)
        ttk.Button(motor_control, text="↓", command=lambda: self.send_motor_command('D')).grid(row=2, column=1)
        ttk.Button(motor_control, text="←", command=lambda: self.send_motor_command('L')).grid(row=1, column=0)
        ttk.Button(motor_control, text="→", command=lambda: self.send_motor_command('R')).grid(row=1, column=2)
        ttk.Button(motor_control, text="Stop", command=lambda: self.send_motor_command('C')).grid(row=1, column=1)
        
        # Pump Control
        pump_control = ttk.LabelFrame(self.control_frame, text="Pump Control", padding="5")
        pump_control.grid(row=3, column=0, padx=5, pady=5, sticky="ew")
        
        self.pump_status = ttk.Label(pump_control, text="Pump Status: Inactive", foreground="red")
        self.pump_status.grid(row=0, column=0, padx=5, pady=5)
        
        ttk.Label(pump_control, text="Press 'T' to toggle pump").grid(row=1, column=0, padx=5, pady=5)

    def setup_key_bindings(self):
        self.root.bind('<Up>', lambda e: self.send_motor_command('U'))
        self.root.bind('<Down>', lambda e: self.send_motor_command('D'))
        self.root.bind('<Left>', lambda e: self.send_motor_command('L'))
        self.root.bind('<Right>', lambda e: self.send_motor_command('R'))
        self.root.bind('<space>', lambda e: self.send_motor_command('C'))
        self.root.bind('t', lambda e: self.toggle_pump())
        self.root.bind('T', lambda e: self.toggle_pump())


    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = glob.glob('/dev/tty.*')
        arduino_ports = [p for p in ports if ('usbmodem' in p.lower() or 'usbserial' in p.lower())]
        self.port_combo['values'] = arduino_ports
        if arduino_ports:
            self.port_combo.set(arduino_ports[0])

    def send_motor_command(self, command):
        if self.is_connected and self.serial:
            try:
                self.serial.write(command.encode())
                self.status_var.set(f"Sent command: {command}")
            except serial.SerialException as e:
                self.status_var.set(f"Error sending command: {str(e)}")
                self.toggle_connection()  # Disconnect on error
        else:
            self.status_var.set("Not connected to device")

    def toggle_pump(self):
        if not self.is_connected:
            self.status_var.set("Cannot toggle pump: Not connected")
            return
            
        self.is_pump_active = not self.is_pump_active
        if self.is_pump_active:
            self.pump_status.configure(text="Pump Status: Active", foreground="green")
            if self.serial:
                self.serial.write(b'T')
        else:
            self.pump_status.configure(text="Pump Status: Inactive", foreground="red")
            if self.serial:
                self.serial.write(b'T')

    def update_camera(self):
        """Camera update loop with proper resource management"""
        while self.is_camera_active:
            with self.camera_lock:
                if self.camera is None or not self.camera.isOpened():
                    break
                    
                ret, frame = self.camera.read()
                if ret:
                    try:
                        # Convert frame to PhotoImage
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = cv2.resize(frame, (640, 480))
                        image = Image.fromarray(frame)
                        photo = ImageTk.PhotoImage(image=image)
                        
                        # Update label
                        self.camera_label.configure(image=photo)
                        self.camera_label.image = photo
                    except Exception as e:
                        print(f"Error processing frame: {e}")
                        break
                
            time.sleep(0.03)  # Limit frame rate

    def stop_camera(self):
        """Safely stop the camera"""
        self.is_camera_active = False
        
        # Wait for camera thread to finish
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
            
        with self.camera_lock:
            if self.camera is not None:
                self.camera.release()
                self.camera = None
                
        # Clear the camera display
        self.camera_label.configure(image='')
        self.camera_label.image = None
        
        # Update button state
        self.camera_btn.configure(text="Start Camera")
        self.status_var.set("Camera stopped")

    def start_camera(self):
        """Safely start the camera"""
        try:
            with self.camera_lock:
                self.camera = cv2.VideoCapture(0)
                if not self.camera.isOpened():
                    raise Exception("Could not open camera")
                
                self.is_camera_active = True
                self.camera_btn.configure(text="Stop Camera")
                
                # Start camera thread
                self.camera_thread = threading.Thread(target=self.update_camera)
                self.camera_thread.daemon = True
                self.camera_thread.start()
                
                self.status_var.set("Camera started")
                
        except Exception as e:
            self.status_var.set(f"Error: Could not open camera - {str(e)}")
            self.stop_camera()

    def toggle_camera(self):
        """Toggle camera state with proper error handling"""
        if not self.is_camera_active:
            self.start_camera()
        else:
            self.stop_camera()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.cleanup)
    root.mainloop()