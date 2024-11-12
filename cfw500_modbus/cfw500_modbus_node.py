import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient as ModbusClient
from std_msgs.msg import Float32

class CFW500ModbusNode(Node):
    def __init__(self):
        super().__init__('cfw500_modbus_node')
        
        # Configurações do Modbus RTU
        self.client = ModbusClient(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

        # Tente conectar ao inversor
        if not self.client.connect():
            self.get_logger().error('Could not connect to Modbus device')
            raise Exception("Modbus connection failed")
        
        # Define o ID do dispositivo (endereço do inversor)
        self.device_id = 1
        self.client.unit_id = self.device_id  # Define o ID do dispositivo no cliente
        
        # Cria publicadores para cada registrador
        self.freq_publisher = self.create_publisher(Float32, 'cfw500_frequency', 10)
        self.vel_publisher = self.create_publisher(Float32, 'cfw500_output_speed', 10)
        self.curr_publisher = self.create_publisher(Float32, 'cfw500_motor_current', 10)
        self.volt_publisher = self.create_publisher(Float32, 'cfw500_output_voltage', 10)
        
        # Define uma taxa para as leituras (em segundos)
        self.timer_period = 1.0  # 1 segundo
        self.timer = self.create_timer(self.timer_period, self.read_and_publish_registers)
        
    def read_and_publish_registers(self):
        try:
            # Lê o registrador de frequência (endereço 1000)
            response = self.client.read_holding_registers(1000, 1)
            if not response.isError():
                frequency = response.registers[0] / 100.0  # Ajuste de escala
                self.publish_data(self.freq_publisher, frequency, "Frequency")
            
            # Lê o registrador de velocidade de saída (P0002)
            response = self.client.read_holding_registers(2, 1)
            if not response.isError():
                output_speed = response.registers[0] / 100.0  # Ajuste de escala
                self.publish_data(self.vel_publisher, output_speed, "Output Speed")
            
            # Lê o registrador de corrente do motor (P0003)
            response = self.client.read_holding_registers(3, 1)
            if not response.isError():
                motor_current = response.registers[0] / 100.0  # Ajuste de escala
                self.publish_data(self.curr_publisher, motor_current, "Motor Current")
            
            # Lê o registrador de tensão de saída (P0007)
            response = self.client.read_holding_registers(7, 1)
            if not response.isError():
                output_voltage = response.registers[0] / 100.0  # Ajuste de escala
                self.publish_data(self.volt_publisher, output_voltage, "Output Voltage")

        except Exception as e:
            self.get_logger().error(f"Error reading Modbus registers: {str(e)}")
    
    def publish_data(self, publisher, data, label):
        self.get_logger().info(f"{label}: {data}")
        msg = Float32()
        msg.data = data
        publisher.publish(msg)

    def destroy(self):
        self.client.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = CFW500ModbusNode()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

