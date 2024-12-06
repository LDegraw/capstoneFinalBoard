import serial
import serial.tools.list_ports
import time

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
        return None
    
    print("\nAvailable ports:")
    for port in ports:
        print(f"- {port.device} ({port.description})")
    return ports

def read_serial(port_name, baud_rate=115200):
    """Read data from the specified serial port"""
    try:
        # Configure serial connection
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        print(f"\nConnected to {port_name} at {baud_rate} baud")
        print("Reading data (Ctrl+C to stop)...\n")
        
        while True:
            if ser.in_waiting:
                # Read line from serial port
                line = ser.readline()
                try:
                    # Try to decode as UTF-8
                    decoded_line = line.decode('utf-8').strip()
                    print(f"Received: {decoded_line}")
                except UnicodeDecodeError:
                    # If UTF-8 fails, print raw bytes
                    print(f"Received (raw): {line}")
            
            time.sleep(0.1)  # Small delay to prevent CPU hogging
            
    except serial.SerialException as e:
        print(f"Error: Could not open port {port_name}: {e}")
    except KeyboardInterrupt:
        print("\nStopping serial reader...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    # First, list all available ports
    ports = list_serial_ports()
    
    if ports:
        # On Mac, USB serial devices typically show up as /dev/tty.usbserial-* or /dev/tty.usbmodem*
        default_port = next((port.device for port in ports if 'usbserial' in port.device or 'usbmodem' in port.device), None)
        
        if default_port:
            print(f"\nFound USB serial device at {default_port}")
            read_serial(default_port)
        else:
            print("\nNo USB serial device automatically detected.")
            port_name = input("Please enter the port name manually (e.g., /dev/tty.usbserial-XXXXX): ")
            read_serial(port_name)