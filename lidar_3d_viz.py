
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import math
import sys

# Configurações Visuais
WINDOW_SIZE = (1000, 800)
GRID_DIM = 20

class LidarVizNode(Node):
    def __init__(self):
        super().__init__('lidar_3d_viz')
        
        # Tópico padrão da bridge (sem namespace por enquanto, ou ajustável)
        # O launch file faz bridge de /x500/lidar_3d/points
        topic_name = '/x500/lidar_3d/points'
        
        # Se quiser suportar namespace no futuro:
        # topic_name = f'/{ns}/x500/lidar_3d/points'
        
        self.get_logger().info(f'Subscribing to: {topic_name}')

        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.listener_callback,
            10)
        
        self.points = [] # Lista de tuplas (x, y, z)
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        # Lê os pontos (x, y, z)
        # skip_nans=True é importante para filtrar pontos inválidos
        gen = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        
        new_points = []
        # Downsample simples para performance (pega 1 a cada 5 pontos se quiser, ou todos)
        # O lidar 3D tem muitos pontos, desenhar linhas para todos pode pesar no PyGame/OpenGL simples
        step = 2 
        for i, p in enumerate(gen):
            if i % step == 0:
                new_points.append((p[0], p[1], p[2]))
        
        with self.lock:
            self.points = new_points

class Visualizer3D:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.running = True

        # Câmera
        self.camera_angle = 45.0
        self.camera_height = 10.0
        self.camera_distance = 15.0
        self.mouse_drag = False
        self.last_mouse_pos = (0, 0)
        
    def init_opengl(self):
        pygame.init()
        pygame.display.set_mode(WINDOW_SIZE, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Lidar 3D Visualizer")
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (WINDOW_SIZE[0]/WINDOW_SIZE[1]), 0.1, 100.0)
        
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def update_camera(self):
        glLoadIdentity()
        
        # Converte polar para cartesiano
        rads = math.radians(self.camera_angle)
        cam_x = math.cos(rads) * self.camera_distance
        cam_y = math.sin(rads) * self.camera_distance
        cam_z = self.camera_height
        
        gluLookAt(cam_x, cam_y, cam_z, 0, 0, 0, 0, 0, 1)

    def draw_grid(self):
        glDisable(GL_LIGHTING)
        glLineWidth(1)
        glColor4f(0.5, 0.5, 0.5, 0.3)
        
        glBegin(GL_LINES)
        d = GRID_DIM
        for i in range(-d, d + 1):
            glVertex3f(i, -d, 0)
            glVertex3f(i, d, 0)
            glVertex3f(-d, i, 0)
            glVertex3f(d, i, 0)
        glEnd()

        # Eixos
        glLineWidth(3)
        glBegin(GL_LINES)
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(2, 0, 0) # X
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 2, 0) # Y
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2) # Z
        glEnd()

    def draw_drone(self):
        # Desenha uma esfera no centro representando o drone
        glColor3f(1.0, 1.0, 0.0) # Amarelo
        sphere = gluNewQuadric()
        gluSphere(sphere, 0.2, 16, 16)

    def draw_rays(self):
        points = []
        with self.ros_node.lock:
            points = list(self.ros_node.points)
            
        if not points:
            return

        glLineWidth(1)
        glBegin(GL_LINES)
        
        # Cor dos raios (Ciano)
        # Pode fazer gradiente baseado na distância se quiser
        
        for p in points:
            x, y, z = p
            dist = math.sqrt(x*x + y*y + z*z)
            
            # Cor baseada na distância (perto = vermelho, longe = azul)
            # Max range esperado ~30m
            ratio = min(dist / 10.0, 1.0)
            glColor3f(1.0 - ratio, ratio, 1.0) # Gradiente Roxo -> Ciano -> Branco

            glVertex3f(0, 0, 0) # Origem (Drone)
            glVertex3f(x, y, z) # Ponto tocado
            
        glEnd()
        
        # Desenha pontos nas pontas para ficar mais visível
        glPointSize(3)
        glBegin(GL_POINTS)
        glColor3f(1, 0, 0) # Ponta vermelha
        for p in points:
            glVertex3f(p[0], p[1], p[2])
        glEnd()

    def run(self):
        self.init_opengl()
        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1: self.mouse_drag = True
                    self.last_mouse_pos = pygame.mouse.get_pos()
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1: self.mouse_drag = False
                elif event.type == pygame.MOUSEMOTION:
                    if self.mouse_drag:
                        x, y = pygame.mouse.get_pos()
                        dx = x - self.last_mouse_pos[0]
                        dy = y - self.last_mouse_pos[1]
                        self.camera_angle -= dx * 0.5
                        self.camera_height += dy * 0.1
                        self.last_mouse_pos = (x, y)
                elif event.type == pygame.MOUSEWHEEL:
                    self.camera_distance -= event.y * 1.0
                    if self.camera_distance < 2.0: self.camera_distance = 2.0

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            self.update_camera()
            self.draw_grid()
            self.draw_drone()
            self.draw_rays()

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    
    # Inicia o Node ROS em uma thread separada
    node = LidarVizNode()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.daemon = True
    ros_thread.start()

    # Inicia visualização na thread principal (PyGame precisa ser Main Thread)
    viz = Visualizer3D(node)
    try:
        viz.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
