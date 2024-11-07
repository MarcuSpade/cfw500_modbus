import rclpy
from rclpy.node import Node
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from std_msgs.msg import Float32

class CFW500ModbusNode(Node):
    def __init__(self):
        super().__init__('cfw500_modbus_node')
        
        # Configurações do Modbus RTU
        self.client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600, timeout=1)
        
        # Tente conectar ao inversor
        if not self.client.connect():
            self.get_logger().error('Could not connect to Modbus device')
            raise Exception("Modbus connection failed")
        
        # Define o ID do dispositivo (endereço do inversor)
        self.device_id = 1
        
        # Cria o publicador para os dados lidos
        self.publisher = self.create_publisher(Float32, 'cfw500_data', 10)
        
        # Define uma taxa para as leituras (em segundos)
        self.timer_period = 1.0  # 1 segundo
        self.timer = self.create_timer(self.timer_period, self.read_and_publish_registers)
        
    def read_and_publish_registers(self):
        try:
            # Lê registrador do inversor (exemplo: registrador de frequência de saída no endereço 1000)
            response = self.client.read_holding_registers(1000, 1, unit=self.device_id)
            if not response.isError():
                # Extrai o valor do registrador
                frequency = response.registers[0] / 100.0  # Exemplo: ajuste do valor
                self.get_logger().info(f"Frequency: {frequency} Hz")
                
                # Publica no tópico
                msg = Float32()
                msg.data = frequency
                self.publisher.publish(msg)
            else:
                self.get_logger().error('Failed to read register')
                
        except Exception as e:
            self.get_logger().error(f"Error reading Modbus register: {str(e)}")
    
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
