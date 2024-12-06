import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import glob

class ServoControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Joint Control")
        
        # Find and connect to the serial port
        self.connect_serial()

        # Create main frame
        self.frame = ttk.Frame(root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Add refresh port button
        ttk.Button(self.frame, text="Refresh Port", command=self.connect_serial).grid(row=0, column=0, columnspan=2, pady=5)
        
        # Port status label
        self.status_label = ttk.Label(self.frame, text="Not Connected")
        self.status_label.grid(row=0, column=2, pady=5)

        # Create sliders and labels
        self.create_joint_control(1, "Joint 2&6 (0-1023)")
        self.create_motor_control(2, "Motor 8 (0-1023)")

        # Update button
        ttk.Button(self.frame, text="Update Motors", command=self.update_motors).grid(row=3, column=0, columnspan=2, pady=10)

        # Store current values
        self.joint_value = 512  # Start at middle position
        self.motor8_value = 0

    def connect_serial(self):
        try:
            # Close existing connection if any
            if hasattr(self, 'ser') and self.ser is not None:
                self.ser.close()

            # Find USB-CDC port on Mac
            ports = list(serial.tools.list_ports.comports())
            usb_port = None
            
            # Look for USB CDC device
            for port in ports:
                if 'usbmodem' in port.device.lower():
                    usb_port = port.device
                    break
            
            if usb_port is None:
                raise Exception("No USB CDC device found")

            self.ser = serial.Serial(usb_port, 115200, timeout=1)
            if hasattr(self, 'status_label'):
                self.status_label.config(text=f"Connected: {usb_port}")
            print(f"Connected to {usb_port}")
            
        except Exception as e:
            if hasattr(self, 'status_label'):
                self.status_label.config(text="Not Connected")
            print(f"Failed to connect: {e}")
            self.ser = None
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")

    def create_joint_control(self, index, label):
        ttk.Label(self.frame, text=label).grid(row=index, column=0, padx=5, pady=5)
        slider = ttk.Scale(self.frame, from_=0, to=1023, orient=tk.HORIZONTAL, length=200)
        slider.set(512)  # Set to middle position
        slider.grid(row=index, column=1, padx=5, pady=5)
        
        # Create two value labels for both motors
        motor2_label = ttk.Label(self.frame, text="Motor 2: 512")
        motor2_label.grid(row=index, column=2, padx=5, pady=5)
        motor6_label = ttk.Label(self.frame, text="Motor 6: 512")
        motor6_label.grid(row=index, column=3, padx=5, pady=5)
        
        def update_label(value):
            value = int(float(value))
            self.joint_value = value
            # Motor 6 gets opposite value (1023 - value)
            motor2_label.configure(text=f"Motor 2: {value}")
            motor6_label.configure(text=f"Motor 6: {1023-value}")
        
        slider.configure(command=update_label)

    def create_motor_control(self, index, label):
        ttk.Label(self.frame, text=label).grid(row=index, column=0, padx=5, pady=5)
        slider = ttk.Scale(self.frame, from_=0, to=1023, orient=tk.HORIZONTAL, length=200)
        slider.grid(row=index, column=1, padx=5, pady=5)
        value_label = ttk.Label(self.frame, text="0")
        value_label.grid(row=index, column=2, padx=5, pady=5)
        
        def update_label(value):
            value = int(float(value))
            value_label.configure(text=f"{value}")
            self.motor8_value = value
        
        slider.configure(command=update_label)

    def update_motors(self):
        if self.ser is None:
            messagebox.showerror("Error", "No serial connection")
            return

        try:
            # Create command packet
            command = bytearray(7)  # 7 bytes total
            command[0] = ord('S')  # Command byte for servo control
            
            # Motor 2 angle
            command[2] = (self.joint_value >> 8) & 0xFF      # High byte
            command[1] = self.joint_value & 0xFF             # Low byte
            
            # Motor 6 angle (inverse of motor 2)
            inverse_value = 1023 - self.joint_value
            command[4] = (inverse_value >> 8) & 0xFF         # High byte
            command[3] = inverse_value & 0xFF                # Low byte
            
            # Motor 8 angle
            command[6] = (self.motor8_value >> 8) & 0xFF     # High byte
            command[5] = self.motor8_value & 0xFF            # Low byte

            self.ser.write(command)
            print(f"Sent command: {[hex(x) for x in command]}")
            
        except Exception as e:
            print(f"Failed to send command: {e}")
            messagebox.showerror("Error", f"Failed to send command: {e}")
            self.connect_serial()  # Try to reconnect

def main():
    root = tk.Tk()
    app = ServoControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()