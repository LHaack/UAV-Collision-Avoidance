import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import asyncio
import websockets
import json
import threading
import math
import time

# Configurações Visuais
WINDOW_SIZE = (1000, 800)
GRID_DIM = 15
WALL_ALPHA = 0.3 # Transparência das paredes

# Cores distintas para drones (R, G, B)
DRONE_COLORS = [
    (0.0, 0.0, 1.0), # Drone 0: Azul
    (0.0, 1.0, 1.0), # Drone 1: Ciano
    (1.0, 0.0, 1.0), # Drone 2: Magenta
    (1.0, 1.0, 0.0), # Drone 3: Amarelo
    (0.5, 0.0, 1.0), # Drone 4: Roxo
    (1.0, 0.5, 0.0)  # Drone 5: Laranja
]

class Visualizer3D:
    def draw_cube(self, x, y, z, size, color, alpha=1.0):
        glPushMatrix()
        glTranslatef(x, y, z)
        
        # Habilita Blend se alpha < 1
        if alpha < 1.0:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glColor4f(color[0], color[1], color[2], alpha)
        
        # Desenha um cubo sólido (Voxel)
        hs = size / 2.0 
        
        glBegin(GL_QUADS)
        # Frente
        glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(-hs, -hs,  hs); glVertex3f( hs, -hs,  hs)
        glVertex3f( hs,  hs,  hs); glVertex3f(-hs,  hs,  hs)
        # Trás
        glNormal3f(0.0, 0.0, -1.0);
        glVertex3f(-hs, -hs, -hs); glVertex3f(-hs,  hs, -hs)
        glVertex3f( hs,  hs, -hs); glVertex3f( hs, -hs, -hs)
        # Esquerda
        glNormal3f(-1.0, 0.0, 0.0);
        glVertex3f(-hs, -hs, -hs); glVertex3f(-hs, -hs,  hs)
        glVertex3f(-hs,  hs,  hs); glVertex3f(-hs,  hs, -hs)
        # Direita
        glNormal3f(1.0, 0.0, 0.0);
        glVertex3f( hs, -hs, -hs); glVertex3f( hs,  hs, -hs)
        glVertex3f( hs,  hs,  hs); glVertex3f( hs, -hs,  hs)
        # Topo
        glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(-hs,  hs, -hs); glVertex3f(-hs,  hs,  hs)
        glVertex3f( hs,  hs,  hs); glVertex3f( hs,  hs, -hs)
        # Base
        glNormal3f(0.0, -1.0, 0.0);
        glVertex3f(-hs, -hs, -hs); glVertex3f( hs, -hs, -hs)
        glVertex3f( hs, -hs,  hs); glVertex3f(-hs, -hs,  hs)
        glEnd()
        
        if alpha < 1.0:
            glDisable(GL_BLEND)
        
        glPopMatrix()

    def __init__(self):
        self.data = None
        self.drone_paths = {} # Changed from list to dict for multi-robot support
        self.lock = threading.Lock()
        self.running = True
        self.connected = False 

        # --- CONFIGURAÇÃO DA CÂMERA ---
        self.CAMERA_ROTATION = True  # <--- Mude para False se quiser estática
        self.camera_angle = 0.0      # Ângulo atual
        self.camera_speed = 0.5      # Velocidade da rotação (graus por frame)
        self.camera_radius = 35.0    # Distância da câmera ao centro
        self.camera_height = 25.0    # Altura da câmera (Z)
        # ------------------------------

        # Cores (R, G, B)
        self.COLOR_BG = (1.0, 1.0, 1.0)
        self.COLOR_GRID = (0.7, 0.7, 0.7)
        self.COLOR_DRONE = (0.0, 0.0, 1.0) # Azul
        self.COLOR_TARGET = (1.0, 0.0, 0.0) # Vermelho
        self.COLOR_OBSTACLE = (1.0, 0.6, 0.0) # Laranja

    def init_opengl(self):
        pygame.init()
        pygame.display.set_mode(WINDOW_SIZE, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Drone 3D - Aguardando conexão...")
        
        # Configuração da Matriz de Projeção (Lente da Câmera)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (WINDOW_SIZE[0]/WINDOW_SIZE[1]), 0.1, 100.0)
        
        # Volta para ModelView para desenhar objetos
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE) # Otimização: Não desenha as faces de trás (reduz poluição visual na transparência)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        # GL_LIGHT1 removida para evitar achatamento do contraste
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Luz Principal
        glLightfv(GL_LIGHT0, GL_POSITION, (10, 10, 10, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))
        
        # Luz Ambiente Global (Levanta as sombras uniformemente sem criar brilho direcional)
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, (0.5, 0.5, 0.5, 1.0))

    def update_camera(self):
        """Calcula e aplica a posição da câmera para este frame."""
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity() # Reseta a matriz para não acumular transformações
        
        # HEADLIGHT SYSTEM: Luz posicionada na origem da câmera (antes de aplicar a transformação de visão)
        # Isso garante que a luz sempre venha "dos olhos" do observador, garantindo iluminação constante.
        glLightfv(GL_LIGHT0, GL_POSITION, (0, 0, 0, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.8, 0.8, 0.8, 1.0))
        
        center_x = 7.5
        center_y = 7.5
        center_z = 7.5

        if self.CAMERA_ROTATION:
            # Incrementa o ângulo
            self.camera_angle += self.camera_speed
            if self.camera_angle >= 360: self.camera_angle -= 360

            # Converte polar para cartesiano para orbitar
            rads = math.radians(self.camera_angle)
            cam_x = center_x + math.cos(rads) * self.camera_radius
            cam_y = center_y + math.sin(rads) * self.camera_radius
            cam_z = self.camera_height
        else:
            # Posição Estática (Diagonal fixa)
            cam_x = -10
            cam_y = -10
            cam_z = 25

        # Aplica a câmera
        gluLookAt(
            cam_x, cam_y, cam_z,   # Onde a câmera está
            center_x, center_y, center_z, # Para onde ela olha (Centro do Grid)
            0, 0, 1                # Onde é "Cima" (Eixo Z)
        )

    def draw_voxel_walls(self, walls_list, color, alpha=1.0):
        if not walls_list: return
        
        # Convert to set for O(1) lookup
        # Coordinates are likely lists, need tuples for set
        walls_set = set(tuple(w) for w in walls_list)
        
        glPushMatrix()
        # Disable lighting for transparency blending smoothness? Or keep it?
        # Keeping lighting allows 3D depth perception. 
        
        # Enable Blend
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # Disable Depth Write for visible transparency (prevents "blades"/hiding behind itself)
        glDepthMask(GL_FALSE)
        
        glColor4f(color[0], color[1], color[2], alpha)
        
        hs = 0.5 # Half size (assuming size 1.0)
        
        glBegin(GL_QUADS)
        
        for w in walls_list:
            cx, cy, cz = w[0], w[1], w[2]
            
            # Check 6 neighbors
            # Top (0, 1, 0)
            if (cx, cy + 1, cz) not in walls_set:
                glNormal3f(0.0, 1.0, 0.0)
                glVertex3f(cx-hs, cy+hs, cz-hs); glVertex3f(cx-hs, cy+hs, cz+hs)
                glVertex3f(cx+hs, cy+hs, cz+hs); glVertex3f(cx+hs, cy+hs, cz-hs)
                
            # Bottom (0, -1, 0)
            if (cx, cy - 1, cz) not in walls_set:
                glNormal3f(0.0, -1.0, 0.0)
                glVertex3f(cx-hs, cy-hs, cz-hs); glVertex3f(cx+hs, cy-hs, cz-hs)
                glVertex3f(cx+hs, cy-hs, cz+hs); glVertex3f(cx-hs, cy-hs, cz+hs)

            # Front (0, 0, 1)
            if (cx, cy, cz + 1) not in walls_set:
                glNormal3f(0.0, 0.0, 1.0)
                glVertex3f(cx-hs, cy-hs, cz+hs); glVertex3f(cx+hs, cy-hs, cz+hs)
                glVertex3f(cx+hs, cy+hs, cz+hs); glVertex3f(cx-hs, cy+hs, cz+hs)

            # Back (0, 0, -1)
            if (cx, cy, cz - 1) not in walls_set:
                glNormal3f(0.0, 0.0, -1.0)
                glVertex3f(cx-hs, cy-hs, cz-hs); glVertex3f(cx-hs, cy+hs, cz-hs)
                glVertex3f(cx+hs, cy+hs, cz-hs); glVertex3f(cx+hs, cy-hs, cz-hs)

            # Right (1, 0, 0)
            if (cx + 1, cy, cz) not in walls_set:
                glNormal3f(1.0, 0.0, 0.0)
                glVertex3f(cx+hs, cy-hs, cz-hs); glVertex3f(cx+hs, cy+hs, cz-hs)
                glVertex3f(cx+hs, cy+hs, cz+hs); glVertex3f(cx+hs, cy-hs, cz+hs)

            # Left (-1, 0, 0)
            if (cx - 1, cy, cz) not in walls_set:
                glNormal3f(-1.0, 0.0, 0.0)
                glVertex3f(cx-hs, cy-hs, cz-hs); glVertex3f(cx-hs, cy-hs, cz+hs)
                glVertex3f(cx-hs, cy+hs, cz+hs); glVertex3f(cx-hs, cy+hs, cz-hs)
        
        glEnd()
        glDepthMask(GL_TRUE) # Re-enable z-buffer writing
        glDisable(GL_BLEND)
        glPopMatrix()

    def draw_sphere(self, x, y, z, radius, color):
        glPushMatrix()
        glTranslatef(x, y, z)
        glColor3f(*color)
        sphere = gluNewQuadric()
        gluSphere(sphere, radius, 16, 16)
        glPopMatrix()

    def draw_grid_box(self):
        glDisable(GL_LIGHTING)
        
        # Grid Fino
        glColor3f(*self.COLOR_GRID)
        glLineWidth(1)
        glBegin(GL_LINES)
        corners = [(0,0,0), (15,0,0), (15,15,0), (0,15,0), (0,0,15), (15,0,15), (15,15,15), (0,15,15)]
        edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)]
        for e in edges:
            glVertex3f(*corners[e[0]])
            glVertex3f(*corners[e[1]])
        glEnd()

        # Eixos
        glLineWidth(3)
        glBegin(GL_LINES)
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(5, 0, 0) # X
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 5, 0) # Y
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 5) # Z
        glEnd()
        glLineWidth(1)

        # Chão Transparente
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(0.8, 0.8, 0.8, 0.3)
        glBegin(GL_LINES)
        for i in range(GRID_DIM + 1):
            glVertex3f(i, 0, 0); glVertex3f(i, GRID_DIM, 0)
            glVertex3f(0, i, 0); glVertex3f(GRID_DIM, i, 0)
        glEnd()
        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)

    def draw_path(self):
        if not self.drone_paths: return
        glDisable(GL_LIGHTING)
        glLineWidth(2)
        
        for robot_id, path in self.drone_paths.items():
            if len(path) < 2: continue
            # Escolhe cor baseada no ID
            color = DRONE_COLORS[robot_id % len(DRONE_COLORS)]
            glColor3f(*color)
            
            glBegin(GL_LINE_STRIP)
            for p in path:
                glVertex3f(p[0], p[1], p[2])
            glEnd()
            
        glEnable(GL_LIGHTING)

    def draw_legend(self):
        # Configuração do HUD 2D
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluOrtho2D(0, WINDOW_SIZE[0], 0, WINDOW_SIZE[1])
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        font = pygame.font.SysFont('Arial', 14, bold=True)
        title_font = pygame.font.SysFont('Arial', 16, bold=True)
        
        # Dimensões e Layout
        panel_width = 160
        item_height = 25
        padding = 10
        title_height = 30
        
        # Identificar itens a mostrar
        items = []
        items.append({"label": "Target", "color": self.COLOR_TARGET})
        items.append({"label": "Obstacle", "color": self.COLOR_OBSTACLE})
        
        # Drones ativos (baseado na lista de cores fixa ou dados recebidos)
        # Vamos listar todos os possíveis da lista de cores para a legenda ficar completa
        for i, color in enumerate(DRONE_COLORS):
             # Opcional: mostrar apenas se existir no 'self.data'
             # Para ficar igual ao pedido (fixo), mostramos todos ou os primeiros N
             if self.data and "robots" in self.data:
                 if not any(r['id'] == i for r in self.data["robots"]):
                     continue
             items.append({"label": f"Drone {i}", "color": color})

        panel_height = title_height + (len(items) * item_height) + padding
        
        start_x = WINDOW_SIZE[0] - panel_width - 20
        start_y = 20 # Bottom aligned anchor (OpenGL coords 0,0 is bottom-left)
        
        # OpenGL coordinates: Y goes UP. 
        # So separate start_y logic:
        # Box Bottom-Left: (start_x, start_y)
        # Box Top-Right: (start_x + panel_width, start_y + panel_height)
        
        # 1. Background Box (White filled)
        glColor4f(1.0, 1.0, 1.0, 1.0) # White
        glBegin(GL_QUADS)
        glVertex2f(start_x, start_y)
        glVertex2f(start_x + panel_width, start_y)
        glVertex2f(start_x + panel_width, start_y + panel_height)
        glVertex2f(start_x, start_y + panel_height)
        glEnd()
        
        # 2. Box Border (Black line)
        glColor3f(0.0, 0.0, 0.0) # Black
        glLineWidth(2)
        glBegin(GL_LINE_LOOP)
        glVertex2f(start_x, start_y)
        glVertex2f(start_x + panel_width, start_y)
        glVertex2f(start_x + panel_width, start_y + panel_height)
        glVertex2f(start_x, start_y + panel_height)
        glEnd()
        glLineWidth(1)
        
        # Helper Text
        def draw_text_gl(x, y, text, font_obj, color=(0,0,0)):
            surface = font_obj.render(text, True, color)
            data = pygame.image.tostring(surface, "RGBA", True)
            glRasterPos2f(x, y)
            glDrawPixels(surface.get_width(), surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data)

        # 3. Title "ENTIDADES"
        # Position: Top of box, centered horizontally
        title_y = start_y + panel_height - 22
        draw_text_gl(start_x + padding, title_y, "ENTIDADES", title_font, (0,0,0))
        
        # 4. Items
        current_y = title_y - item_height
        box_size = 15
        
        for item in items:
            # Color Box
            bx = start_x + padding
            by = current_y + 4 # tiny offset
            
            # Fill
            glColor3f(*item["color"])
            glBegin(GL_QUADS)
            glVertex2f(bx, by); glVertex2f(bx + box_size, by)
            glVertex2f(bx + box_size, by + box_size); glVertex2f(bx, by + box_size)
            glEnd()
            
            # Border
            glColor3f(0,0,0)
            glBegin(GL_LINE_LOOP)
            glVertex2f(bx, by); glVertex2f(bx + box_size, by)
            glVertex2f(bx + box_size, by + box_size); glVertex2f(bx, by + box_size)
            glEnd()
            
            # Label
            draw_text_gl(bx + box_size + 8, by+2, item["label"], font, (0,0,0))
            
            current_y -= item_height

        glDisable(GL_BLEND)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    async def websocket_listener(self):
        uri = "ws://localhost:8765"
        while self.running:
            try:
                print(f"Tentando conectar a {uri}...")
                async with websockets.connect(uri) as websocket:
                    print("Conectado! Recebendo dados...")
                    self.connected = True
                    pygame.display.set_caption("Drone 3D - CONECTADO")
                    
                    while self.running:
                        try:
                            msg = await websocket.recv()
                            data = json.loads(msg)
                            
                            with self.lock:
                                self.data = data
                                
                                # Se o servidor mandou resetar, limpa o rastro
                                if data.get("reset", False):
                                    self.drone_paths = {}

                                if "robots" in data:
                                    for r in data["robots"]:
                                        rid = r["id"]
                                        pos = r["position"]
                                        
                                        if rid not in self.drone_paths:
                                            self.drone_paths[rid] = []
                                            
                                        # Adiciona ponto se for novo
                                        path = self.drone_paths[rid]
                                        if not path or path[-1] != pos:
                                            path.append(pos)
                        except websockets.exceptions.ConnectionClosed:
                            print("Conexão perdida. Tentando reconectar...")
                            self.connected = False
                            break
            except (ConnectionRefusedError, OSError):
                self.connected = False
                pygame.display.set_caption("Drone 3D - Aguardando servidor...")
                await asyncio.sleep(1) 

    def run(self):
        self.init_opengl()
        
        ws_thread = threading.Thread(target=lambda: asyncio.run(self.websocket_listener()))
        ws_thread.daemon = True
        ws_thread.start()

        clock = pygame.time.Clock()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.CAMERA_ROTATION = not self.CAMERA_ROTATION

            # Limpar tela
            glClearColor(*self.COLOR_BG, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            self.update_camera()
            self.draw_grid_box()

            with self.lock:
                if self.connected and self.data:
                    # Target
                    tgt = self.data.get("target")
                    if tgt: self.draw_sphere(tgt[0], tgt[1], tgt[2], 0.5, self.COLOR_TARGET)
                    
                    # Obstáculos
                    for obs in self.data.get("dynamic_obstacles", []):
                            self.draw_sphere(obs[0], obs[1], obs[2], 0.4, self.COLOR_OBSTACLE)

                    # Drone e Rastro
                    self.draw_path()
                    for r in self.data.get("robots", []):
                        pos = r["position"]
                        rid = r["id"]
                        color = DRONE_COLORS[rid % len(DRONE_COLORS)]
                        self.draw_sphere(pos[0], pos[1], pos[2], 0.4, color)

                    # Paredes (Voxel Meshing - single transparency call)
                    wall_color = (0.3, 0.3, 0.3) 
                    walls = self.data.get("walls", [])
                    self.draw_voxel_walls(walls, wall_color, alpha=WALL_ALPHA)
                        
            # Draw Legend last (on top)
            self.draw_legend()

            pygame.display.flip()
            clock.tick(30) # FPS do visualizador

        pygame.quit() 

if __name__ == "__main__":
    viz = Visualizer3D()
    viz.run()