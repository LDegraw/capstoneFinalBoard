import serial
import serial.tools.list_ports
import sys
import os
import termios
import tty
import select
import time

class KeyReader:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
    
    def __enter__(self):
        tty.setraw(self.fd)
        return self
    
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    
    def read_key(self):
        """Read a keypress and return the command character"""
        char = sys.stdin.read(1)
        if char == '\x1b':  # Escape sequence
            char += sys.stdin.read(2)
            
            # Map arrow keys to commands
            key_map = {
                '[A': 'U',  # Up arrow
                '[B': 'D',  # Down arrow
                '[C': 'R',  # Right arrow
                '[D': 'L'   # Left arrow
            }
            return key_map.get(char[1:], None)
        elif char in ['w', 'W']:
            return 'U'
        elif char in ['s', 'S']:
            return 'D'
        elif char in ['d', 'D']:
            return 'R'
        elif char in ['a', 'A']:
            return 'L'
        elif char in ['q', 'Q']:
            return 'QUIT'
        return None

def find_stm32_port():
    """Find the STM32 device port automatically"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "STM32" in port.description:
            print(f"Found STM32 device at: {port.device}")
            return port.device
    return None

def setup_serial_connection(port=None, baudrate=115200):
    """Setup serial connection with specified parameters"""
    if port is None:
        port = find_stm32_port()
        if not port:
            print("Error: No STM32 device found!")
            return None
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=1
        )
        
        if not ser.is_open:
            ser.open()
        
        return ser
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("\nTroubleshooting steps:")
        print("1. Check if another program is using the port")
        print("2. Try unplugging and reconnecting the device")
        print("3. Verify you have the correct drivers installed")
        return None

def send_command_sequence(ser, command):
    """Send movement command followed by close command"""
    if not ser or not ser.is_open:
        print("Serial connection lost!")
        return False
        
    try:
        # Send movement command
        ser.write(command.encode())
        print(f"Sent: {command}")
        
        # Wait a short time before sending close command
        time.sleep(0.3)  # 100ms delay
        
        # Send close command
        ser.write(b'C')
        print("Sent: C")
        return True
    except Exception as e:
        print(f"Error sending data: {e}")
        return False

def main():
    ser = setup_serial_connection()
    if not ser:
        return
    
    print("\nSerial connection established")
    print("Use arrow keys or WASD to send commands:")
    print("↑/W: 'U' followed by 'C'")
    print("↓/S: 'D' followed by 'C'")
    print("←/A: 'L' followed by 'C'")
    print("→/D: 'R' followed by 'C'")
    print("Press 'q' to exit")
    
    last_command = None
    error_count = 0
    
    try:
        with KeyReader() as reader:
            while True:
                if error_count > 5:
                    print("\nToo many errors. Reconnecting...")
                    ser.close()
                    time.sleep(1)
                    ser = setup_serial_connection()
                    if not ser:
                        break
                    error_count = 0
                
                if select.select([sys.stdin], [], [], 0.1)[0]:  # 100ms timeout
                    command = reader.read_key()
                    
                    if command == 'QUIT':
                        break
                    
                    if command and command != last_command:
                        if send_command_sequence(ser, command):
                            last_command = command
                            error_count = 0
                        else:
                            error_count += 1
                else:
                    last_command = None
                
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("\nSerial connection closed")

if __name__ == "__main__":
    main()