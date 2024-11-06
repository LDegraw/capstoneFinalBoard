import serial
import keyboard
import time
import serial.tools.list_ports
import sys
import os

def list_available_ports():
    """List all available COM ports"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found!")
        return []
    
    print("\nAvailable COM ports:")
    for port in ports:
        print(f"- {port.device}: {port.description}")
    return [port.device for port in ports]

def setup_serial_connection(port='COM3', baudrate=115200):
    """
    Setup serial connection with specified parameters
    Returns serial object if successful, None if failed
    """
    # Check if running with admin privileges
    is_admin = os.getuid() == 0 if hasattr(os, 'getuid') else True
    if not is_admin:
        print("Warning: Script may need admin privileges. Try running as administrator.")
    
    try:
        # Check if port exists
        available_ports = list_available_ports()
        if port not in available_ports:
            print(f"\nError: {port} not found!")
            if available_ports:
                print(f"Please choose from available ports listed above.")
            return None
        
        # Try to close any existing connections
        try:
            test_ser = serial.Serial(port)
            test_ser.close()
        except:
            pass
        
        # Open new connection
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=1
        )
        
        # Test the connection
        if not ser.is_open:
            ser.open()
        
        # Try to send a test byte
        try:
            ser.write(b'\x00')
            time.sleep(0.1)
            ser.reset_output_buffer()
        except Exception as e:
            print(f"Warning: Initial write test failed: {e}")
            print("Continuing anyway...")
        
        return ser
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("\nTroubleshooting steps:")
        print("1. Run the script as administrator")
        print("2. Check if another program is using the port")
        print("3. Try unplugging and reconnecting the device")
        print("4. Check Device Manager to verify port number")
        return None

def send_serial_data(ser, data):
    """
    Send single character over serial connection
    """
    if not ser or not ser.is_open:
        print("Serial connection lost!")
        return False
        
    try:
        # Convert to bytes if string
        if isinstance(data, str):
            data = data.encode()
        ser.write(data)
        return True
    except serial.SerialTimeoutException:
        print("Write timeout occurred. Device not responding.")
        return False
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False
    except Exception as e:
        print(f"Error sending data: {e}")
        return False

def main():
    # Ask user for COM port
    port = input("Enter COM port (default COM3): ").strip() or 'COM3'
    
    # Setup serial connection
    ser = setup_serial_connection(port)
    if not ser:
        return
    
    print("\nSerial connection established")
    print("Use arrow keys to control")
    print("Press 'esc' to exit")
    
    # Define key mappings - using bytes instead of strings
    key_mappings = {
        'up': b'U',    # SOH
        'down': b'D',  # STX
        'left': b'L',  # ETX
        'right': b'R'  # EOT
    }
    
    # Track last key state to prevent repeated sends
    last_key = None
    error_count = 0
    
    try:
        while True:
            if error_count > 5:
                print("\nToo many errors. Reconnecting...")
                ser.close()
                time.sleep(1)
                ser = setup_serial_connection(port)
                if not ser:
                    break
                error_count = 0
            
            # Check for exit condition
            if keyboard.is_pressed('esc'):
                break
            
            # Check arrow keys
            for key in key_mappings:
                if keyboard.is_pressed(key):
                    current_key = key_mappings[key]
                    if current_key != last_key:
                        if send_serial_data(ser, current_key):
                            print(f"Sent: {key}")
                            last_key = current_key
                            error_count = 0
                        else:
                            error_count += 1
                    break
            else:
                last_key = None
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()