import serial
import time


class VN200:
    def __init__(self, com_port, baudrate=115200, timeout=1):
        """
        Initialize connection to VN200 IMU.
        
        Args:
            com_port (str): COM port identifier (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baudrate (int): Communication baud rate (default: 115200)
            timeout (float): Serial timeout in seconds (default: 1)
        """
        self.com_port = com_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        
        try:
            self.serial_conn = serial.Serial(
                port=com_port,
                baudrate=baudrate,
                timeout=timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print(f"Connected to VN200 on {com_port} at {baudrate} baud")
            time.sleep(0.1)  # Allow time for connection to stabilize
            
        except serial.SerialException as e:
            print(f"Error connecting to VN200: {e}")
            raise
    
    def ASCII_Send(self, command):
        """
        Send ASCII command to VN200.
        
        Args:
            command (str): ASCII command string (without checksum or line endings)
        
        Returns:
            str: Response from VN200 if available, None otherwise
        """
        if self.serial_conn is None or not self.serial_conn.is_open:
            print("Serial connection not open")
            return None
        
        # Ensure command doesn't already have line endings
        command = command.strip()
        
        # VN200 expects commands ending with \r\n
        full_command = command + '\r\n'
        
        try:
            # Clear input buffer
            self.serial_conn.reset_input_buffer()
            
            # Send command
            self.serial_conn.write(full_command.encode('ascii'))
            print(f"Sent: {command}")
            
            # Wait briefly for response
            time.sleep(0.05)
            
            # Read response if available
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode('ascii').strip()
                print(f"Received: {response}")
                print()  # Blank line for readability
                return response
            else:
                print("Received: (no response)")
                print()  # Blank line for readability
            
            return None
            
        except Exception as e:
            print(f"Error sending ASCII command: {e}")
            return None
    
    def close(self):
        """Close the serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"Closed connection to {self.com_port}")


# Example usage:
if __name__ == "__main__":
    # Initialize VN200 on COM3 (adjust port as needed)
    vn = VN200('COM3')
    
    # Send a command (example: request model number)
    vn.ASCII_Send("$VNASY,0*XX")  # Disable asynchronous data output
    vn.ASCII_Send("$$VNWRG,44,1,3,5*XX") # enable real time HSI calibration with quick convergence
    vn.ASCII_Send("$VNWRG,57,0,0,-0.2*XX") # GPS Antenna A offset Z = -0.2m
    
    # Close connection when done
    vn.close()