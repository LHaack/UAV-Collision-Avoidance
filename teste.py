import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from px4_msgs.msg import ObstacleDistance # Certifique-se de ter o pacote px4_msgs

class LidarToObstacle(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        
        # Usando o namespace de forma elegante
        # Isso permite que o nó funcione para px4_1, px4_2, etc.
        self.declare_parameter('namespace', 'px4_1')
        ns = self.get_parameter('namespace').get_parameter_value().string_value

        # Inscrição no tópico do LiDAR (usando o caminho que você definiu no SDF)
        self.subscription = self.create_subscription(
            PointCloud2,
            f'/{ns}/x500/lidar_3d',
            self.listener_callback,
            10)

        # Publicador para o PX4
        self.publisher = self.create_publisher(
            ObstacleDistance, 
            f'/{ns}/fmu/in/obstacle_distance', 
            10)

    def listener_callback(self, msg):
        # 1. Inicializa o array com 65535 (indica "sem obstáculo" no PX4)
        # O PX4 espera exatamente 72 setores (5 graus cada para 360°)
        distances = [65535] * 72 
        increment = 5.0 # graus

        # 2. Lê os pontos (x, y, z) da mensagem PointCloud2
        # O field_names filtra apenas o que precisamos para ganhar performance
        points = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)

        for p in points:
            x, y, z = p[0], p[1], p[2]

            # Filtro de "janela" vertical (ignora o chão e o que está muito alto)
            if z < -0.5 or z > 0.5:
                continue

            # Distância euclidiana 3D
            # $$d = \sqrt{x^2 + y^2 + z^2}$$
            dist = math.sqrt(x**2 + y**2 + z**2)

            # Calcula o ângulo e mapeia para 0-360
            angle_rad = math.atan2(y, x)
            angle_deg = math.degrees(angle_rad) % 360

            # Encontra o setor (0 a 71)
            sector = int(angle_deg / increment)
            
            # Atualiza se for o ponto mais próximo encontrado para aquele setor
            dist_cm = int(dist * 100)
            if dist_cm < distances[sector]:
                distances[sector] = dist_cm

        # 3. Monta e envia a mensagem para o PX4
        obs_msg = ObstacleDistance()
        obs_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        obs_msg.frame = 12 # MAV_FRAME_BODY_FRD
        obs_msg.sensor_type = 1 # MAV_DISTANCE_SENSOR_LASER
        obs_msg.distances = [uint16 if uint16 < 65535 else 65535 for uint16 in distances]
        obs_msg.increment = increment
        obs_msg.min_distance = 10 # 10cm (conforme seu SDF)
        obs_msg.max_distance = 3000 # 30m (conforme seu SDF)
        obs_msg.angle_offset = 0.0
        
        self.publisher.publish(obs_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()