import serial
import sys
import tty
import termios
import time
import glob

def list_serial_ports():
    """List all available serial ports on Mac."""
    ports = glob.glob('/dev/tty.*')
    arduino_ports = [p for p in ports if ('usbmodem' in p.lower() or 'usbserial' in p.lower())]
    return arduino_ports

class KeyboardController:
    def __init__(self, port=None, baudrate=115200):  # Updated baudrate to match Arduino
        # Find available ports
        available_ports = list_serial_ports()
        
        if not available_ports:
            print("No Arduino ports found. Please check if your board is connected.")
            print("Available ports:", glob.glob('/dev/tty.*'))
            sys.exit(1)
            
        if port is None:
            port = available_ports[0]  # Use the first available port
            
        # Initialize serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Successfully connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            print("\nAvailable ports:")
            for p in available_ports:
                print(f"- {p}")
            sys.exit(1)
            
        # Key mapping for arrow keys and space
        self.key_mapping = {
            '\x1b[A': 'U',  # Up arrow
            '\x1b[B': 'D',  # Down arrow
            '\x1b[C': 'R',  # Right arrow
            '\x1b[D': 'L',  # Left arrow
            ' ': 'C'        # Spacebar - stop motors
        }

    def get_key(self):
        """Get a single keypress from terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                ch += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def send_command(self, command):
        """Send command to motor board."""
        try:
            self.ser.write(command.encode())
            print(f"Sent command: {command}")
        except serial.SerialException as e:
            print(f"Error sending command: {e}")

    def run(self):
        """Main control loop."""
        print("\nMotor Control Started")
        print("Controls:")
        print("- Arrow keys: U/D/L/R commands for movement")
        print("- Spacebar: Stop motors (C command)")
        print("- 'q': Quit program")
        
        while True:
            key = self.get_key()
            
            if key == 'q':
                print("Quitting program...")
                break
                
            if key in self.key_mapping:
                command = self.key_mapping[key]
                self.send_command(command)
            
            time.sleep(0.1)

    def cleanup(self):
        """Clean up serial connection."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    # List available ports before starting
    print("Searching for Arduino ports...")
    available_ports = list_serial_ports()
    
    if available_ports:
        print("Found the following Arduino ports:")
        for port in available_ports:
            print(f"- {port}")
        print(f"\nUsing port: {available_ports[0]}")
        
        # Create controller instance with the first found port
        controller = KeyboardController(port=available_ports[0])
        
        try:
            controller.run()
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        finally:
            controller.cleanup()
    else:
        print("No Arduino ports found. Please check if:")
        print("1. Your Arduino is connected to the computer")
        print("2. You have installed the correct Arduino drivers")
        print("3. You have the necessary permissions to access the port")