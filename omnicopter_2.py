#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw, PositionNedYaw)

# --- CONFIGURAÇÕES ---
TARGET_NORTH = 0.0
TARGET_EAST = -10.0
TARGET_DOWN = -2.3 # 2 metros de altura (negativo é pra cima) # -2.699 foi o GOAT

SPEED_TAKEOFF = 2.0  # m/s (Rápido)
SPEED_CRUISE = 0.3   # m/s (Bem devagar)

async def run():
    # 1. Conexão com o drone
    # Se estiver rodando no mesmo PC da simulação, use udp://:14540
    drone = System()
    print("Conectando ao drone...")
    await drone.connect(system_address="udp://:14540")

    # Espera a conexão
    print("Aguardando conexão...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Conectado!")
            break

    # 2. Armar
    print("-- Armando")
    await drone.action.arm()

    # 3. Iniciar modo Offboard
    print("-- Iniciando modo Offboard")
    
    # Precisamos enviar um setpoint inicial antes de ligar o offboard
    initial_setpoint = VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    await drone.offboard.set_velocity_ned(initial_setpoint)

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Falha ao iniciar Offboard: {error}")
        return

    # =========================================================
    # FASE 1: DECOLAGEM RÁPIDA (Velocity Control)
    # =========================================================
    print(f"-- Decolando rápido a {SPEED_TAKEOFF} m/s")
    
    # Loop de subida
    async for position in drone.telemetry.position_velocity_ned():
        current_alt = -position.position.down_m # Inverte porque NED down é positivo pra baixo
        
        # Se chegou na altura (com margem de 0.2m)
        if current_alt >= (-TARGET_DOWN - 0.1):
            print("-- Altitude atingida!")
            break
        
        # Manda velocidade negativa (pra cima) em Z
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -SPEED_TAKEOFF, 0.0))
        
        # O loop roda o mais rápido possível, mas não precisamos spammar tanto
        # await asyncio.sleep(0.05) # Opcional

    # =========================================================
    # FASE 2: MOVIMENTO LENTO (Velocity Control)
    # =========================================================
    print(f"-- Indo para ({TARGET_NORTH}, {TARGET_EAST}) a {SPEED_CRUISE} m/s")

    async for position_vel in drone.telemetry.position_velocity_ned():
        current_n = position_vel.position.north_m
        current_e = position_vel.position.east_m
        current_d = position_vel.position.down_m

        # Vetor Distância
        delta_n = TARGET_NORTH - current_n
        delta_e = TARGET_EAST - current_e
        
        distance = math.sqrt(delta_n**2 + delta_e**2)

        # Se chegou perto o suficiente (10cm)
        if distance < 0.1:
            print("-- Chegou no destino!")
            break

        # Normaliza o vetor (Direção)
        norm_n = delta_n / distance
        norm_e = delta_e / distance

        # Aplica a velocidade lenta
        vel_n = norm_n * SPEED_CRUISE
        vel_e = norm_e * SPEED_CRUISE
        
        # Correção P simples para manter a altura cravada em -2.0
        # (Se o drone cair um pouco, isso o puxa pra cima)
        error_z = TARGET_DOWN - current_d
        vel_d = error_z * 1.5 # Ganho P

        # Envia comando
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_n, vel_e, vel_d, 0.0))

    # =========================================================
    # FASE 3: HOLD (Trava Posição)
    # =========================================================
    print("-- Mantendo posição (Hold)")
    
    # Muda para controle de Posição para ficar travado perfeitamente
    # Note que no MAVSDK podemos trocar de Velocity pra Position sem sair do Offboard
    await drone.offboard.set_position_ned(
        PositionNedYaw(TARGET_NORTH, TARGET_EAST, TARGET_DOWN, 0.0)
    )

    # Mantém o script rodando para segurar o drone lá
    await asyncio.sleep(20)
    
    # Opcional: Pousar depois
    # print("-- Pousando")
    # await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())