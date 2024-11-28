import serial
import serial.rs485

ser = serial.rs485.RS485(
    port='/dev/ttyACM0',  # Update this to your port
    baudrate=115200,
    timeout=0.5           # Timeout for reading response
)

ser.rs485_mode = serial.rs485.RS485Settings()


def send_and_receive(command_hex):
    while True:
        try:
            # Write Hex Command
            print(f"Sending: {command_hex.hex()}")
            ser.write(command_hex)

            # Wait for the device to respond
            response = ser.read(10)  # Adjust size based on expected response length
            print(f"Received: {response.hex()}")
            
            return response

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None

# Example: Sending a Command to Set Motor Speed
# Replace this with the actual command for your motor controller # Example command
motor_feeback = bytes.fromhex("01 74 00 00 00 00 00 00 00 04")
response = send_and_receive(motor_feeback)

ser.close()