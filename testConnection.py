import serial
import serial.tools.list_ports
import time
from dyn_ax_12p import dynamixel_AX12P

class DynamixelLEDController:
    def __init__(self, port, baudrate=1000000):
        """
        Initialize the Dynamixel controller
        Args:
            port (str): Serial port
            baudrate (int): Communication speed (default 1M for AX-12A)
        """
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"Successfully connected to {port}")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise

        self.motors = {}  # Dictionary to store motor instances

    def add_motor(self, motor_id):
        """Add a motor with specified ID to the controller"""
        try:
            self.motors[motor_id] = dynamixel_AX12P(motor_id, self.serial_port)
            print(f"Added motor ID: {motor_id}")
            return True
        except Exception as e:
            print(f"Error adding motor {motor_id}: {e}")
            return False

    def set_led(self, motor_id, state):
        """
        Set the LED state of a motor
        Args:
            motor_id (int): ID of the motor
            state (int): 0 for off, 1 for on
        """
        if motor_id not in self.motors:
            print(f"Motor ID {motor_id} not found")
            return False

        motor = self.motors[motor_id]
        try:
            # Address 25 (0x19) is for LED
            motor.init_packet_param_AX_12p([25, state])
            motor.command_to_AX_12p(3)  # Write command
            motor.release_packet_param_AX_12p()
            print(f"LED for motor {motor_id} {'turned ON' if state else 'turned OFF'}")
            return True
        except Exception as e:
            print(f"Error setting LED: {e}")
            return False

    def close(self):
        """Close the serial connection"""
        if self.serial_port.is_open:
            self.serial_port.close()
            print("Serial connection closed")

def list_available_ports():
    """List all available USB serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("\nNo USB serial ports found")
        return []
    
    print("\nAvailable ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    return ports

def print_menu():
    """Print the control menu"""
    print("\n=== Dynamixel LED Control Menu ===")
    print("1. Add motor")
    print("2. Turn LED ON")
    print("3. Turn LED OFF")
    print("4. List connected motors")
    print("5. Exit")
    print("================================")

def get_valid_motor_id():
    """Get a valid motor ID from user input"""
    while True:
        try:
            motor_id = int(input("Enter motor ID (1-253): ").strip())
            if 1 <= motor_id <= 253:
                return motor_id
            print("Invalid motor ID. Please enter a value between 1 and 253.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def main():
    # List available ports
    available_ports = list_available_ports()
    if not available_ports:
        return

    # Port selection
    while True:
        try:
            selection = input("\nEnter the number of the port to use (or 'q' to quit): ")
            if selection.lower() == 'q':
                return
            
            port_index = int(selection)
            if 0 <= port_index < len(available_ports):
                selected_port = available_ports[port_index].device
                break
            print("Invalid selection. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number or 'q' to quit.")

    try:
        # Initialize controller with selected port
        controller = DynamixelLEDController(selected_port)
        
        while True:
            print_menu()
            choice = input("Enter your choice (1-5): ").strip()
            
            if choice == '1':
                motor_id = get_valid_motor_id()
                controller.add_motor(motor_id)
                    
            elif choice == '2':
                motor_id = get_valid_motor_id()
                controller.set_led(motor_id, 1)
                
            elif choice == '3':
                motor_id = get_valid_motor_id()
                controller.set_led(motor_id, 0)

            elif choice == '4':
                if not controller.motors:
                    print("\nNo motors connected yet")
                else:
                    print("\nConnected motors:")
                    for motor_id in controller.motors:
                        print(f"Motor ID: {motor_id}")
                
            elif choice == '5':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please enter a number between 1 and 5.")
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main()