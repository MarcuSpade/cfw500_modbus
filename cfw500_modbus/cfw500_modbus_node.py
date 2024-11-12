import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from std_msgs.msg import Float32

class CFW500ModbusNode(Node):
    def __init__(self):
        super().__init__('cfw500_modbus_node')
        
        # Set up Modbus RTU client configuration
        # Configures the Modbus client for communication with a device on /dev/ttyUSB0
        # using a baud rate of 9600 and a timeout of 1 second.
        self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=19200, timeout=1)

        # Attempt connection to the inverter device via Modbus
        if not self.client.connect():
            self.get_logger().error('Could not connect to Modbus device')
            raise Exception("Modbus connection failed")
        
        # Set device ID (or Modbus unit ID) for the inverter
        self.device_id = 1
        self.client.unit_id = self.device_id  # Assigns unit ID for addressing the device

        # Initialize ROS 2 publishers for each Modbus register
        # These topics will publish frequency, output speed, motor current, and output voltage.
        self.freq_publisher = self.create_publisher(Float32, 'cfw500_frequency', 10)
        self.vel_publisher = self.create_publisher(Float32, 'cfw500_output_speed', 10)
        self.curr_publisher = self.create_publisher(Float32, 'cfw500_motor_current', 10)
        self.volt_publisher = self.create_publisher(Float32, 'cfw500_output_voltage', 10)
        
        # Sets the reading frequency to 1 second intervals
        self.timer_period = 1.0  # 1 second period
        self.timer = self.create_timer(self.timer_period, self.read_and_publish_registers)

    def read_and_publish_registers(self):
        """
        Reads specific Modbus registers and publishes the data to ROS 2 topics.
        Each reading is scaled appropriately before publishing.
        """
        try:
            # Read frequency register (address P0005)
            response = self.client.read_holding_registers(5, 1)
            if not response.isError():
                frequency = response.registers[0] / 100.0  # Scale adjustment
                self.publish_data(self.freq_publisher, frequency, "Frequency")
            
            # Read output speed register (P0002, address 2)
            response = self.client.read_holding_registers(2, 1)
            if not response.isError():
                output_speed = response.registers[0] / 100.0  # Scale adjustment
                self.publish_data(self.vel_publisher, output_speed, "Output Speed")
            
            # Read motor current register (P0003, address 3)
            response = self.client.read_holding_registers(3, 1)
            if not response.isError():
                motor_current = response.registers[0] / 100.0  # Scale adjustment
                self.publish_data(self.curr_publisher, motor_current, "Motor Current")
            
            # Read output voltage register (P0007, address 7)
            response = self.client.read_holding_registers(7, 1)
            if not response.isError():
                output_voltage = response.registers[0] / 100.0  # Scale adjustment
                self.publish_data(self.volt_publisher, output_voltage, "Output Voltage")

        except Exception as e:
            # Logs any error encountered during Modbus register reading
            self.get_logger().error(f"Error reading Modbus registers: {str(e)}")
    
    def publish_data(self, publisher, data, label):
        """
        Logs and publishes data to a specified ROS 2 topic.

        Parameters:
        - publisher (Publisher): The ROS publisher to use.
        - data (float): The data to publish.
        - label (str): The label for logging purposes.
        """
        self.get_logger().info(f"{label}: {data}")
        msg = Float32()
        msg.data = data
        publisher.publish(msg)

    def destroy(self):
        """
        Safely closes the Modbus client connection and calls the superclass
        destroy method for cleanup.
        """
        self.client.close()
        super().destroy()

def main(args=None):
    """
    Main function to initialize and run the ROS node.
    """
    rclpy.init(args=args)
    node = CFW500ModbusNode()
    rclpy.spin(node)  # Keep the node running
    node.destroy()  # Clean up after exiting
    rclpy.shutdown()  # Shut down the ROS client library

if __name__ == '__main__':
    main()
