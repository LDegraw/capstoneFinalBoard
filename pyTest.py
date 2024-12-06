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
        print(f"{port.device} - {port.description}")
    return ports

def read_serial_data(port_name, baud_rate=115200, timeout=1):
    """Read data from the specified serial port"""
    try:
        # Configure serial connection
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        print(f"\nConnected to {port_name} at {baud_rate} baud")
        print("Reading data... (Press Ctrl+C to exit)")
        
        while True:
            if ser.in_waiting:
                # Read line of data
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line:
                        timestamp = time.strftime('%H:%M:%S')
                        print(f"[{timestamp}] {line}")
                except UnicodeDecodeError:
                    print("Error decoding data")
            time.sleep(0.01)  # Reduced sleep time for faster response
                
    except serial.SerialException as e:
        print(f"Error: Could not open port {port_name}: {e}")
    except KeyboardInterrupt:
        print("\nStopping serial reader...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed")

def main():
    # List available ports
    ports = list_serial_ports()
    if not ports:
        return
    
    # If only one port is available, use it automatically
    if len(ports) == 1:
        port_name = ports[0].device
        print(f"\nAutomatic port selection: {port_name}")
    else:
        # Get user input for port selection
        port_name = input("\nEnter the port name (e.g., /dev/tty.usbserial-1420): ")
    
    # Start reading data with 115200 baud rate
    read_serial_data(port_name, baud_rate=115200)

if __name__ == "__main__":
    main()