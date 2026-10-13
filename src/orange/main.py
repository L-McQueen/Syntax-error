"""Orquestador autónomo y punto de entrada para Orange Pi Zero 2W.
Gestiona el menú OLED, selección de pistas (A / B) y parada de emergencia."""

import os
import sys
import time

from menu import MenuManager
from robot_hal import RobotHAL


def run_sensor_diagnostic_mode(hal, menu):
    """Bucle continuo de diagnóstico en pantalla para verificación en pits."""
    print("[MODO] Entrando a Diagnóstico de Sensores en Pits...")
    time.sleep(0.3)
    
    while True:
        # Actualizar pantalla de diagnóstico
        menu.show_sensor_diagnostic(hal)

        # Si se presiona cualquier botón, salir de regreso al menú
        btn = menu.check_buttons()
        if btn is not None:
            print("[MODO] Saliendo del diagnóstico...")
            time.sleep(0.3)
            break

        time.sleep(0.1)


def run_calibrate_imu_mode(hal, menu):
    """Re-calibra el sesgo del giróscopo manteniendo el robot inmóvil."""
    print("[MODO] Re-calibrando IMU...")
    menu.show_splash("Calibrando IMU...")
    time.sleep(0.5)

    if hal.imu_sensor:
        bias_accum = 0.0
        samples = 150
        for _ in range(samples):
            try:
                gyro_z = hal.imu_sensor.get_gyro_data()['z']
                bias_accum += gyro_z
            except Exception:
                pass
            time.sleep(0.01)

        hal.gyro_bias_z = bias_accum / samples
        hal.reset_yaw(0.0)
        print(f"[MODO] Nuevo sesgo IMU Z: {hal.gyro_bias_z:.3f} deg/s")
        menu.show_splash("!IMU Calibrado!")
        time.sleep(1.0)
    else:
        menu.show_splash("IMU no detectado")
        time.sleep(1.0)


def run_track_a(hal, menu):
    """
    Rutina autónoma para Pista A (Navegación de laberinto y lectura ArUco).
    Integra la lógica de Sim2Real probada en Webots con las lecturas reales del HAL.
    """
    print("[MODO] === INICIANDO PISTA A ===")
    menu.show_countdown(3)
    
    # Parámetros de control de navegación
    TARGET_WALL_DIST_CM = 15.0  # Distancia deseada a las paredes laterales
    BASE_SPEED_PWM = 160        # Velocidad base de avance
    KP_WALL = 3.5               # Ganancia proporcional de centrado
    FRONT_COLLISION_CM = 18.0   # Umbral de giro frontal
    
    fsm_state = "AVANCE"
    last_telemetry_time = 0.0
    
    try:
        while True:
            # 1. VERIFICACIÓN CRÍTICA: Parada de emergencia por botón físico
            if menu.is_emergency_pressed():
                print("[EMERGENCIA] ¡Botón de parada presionado por el operador!")
                hal.stop_all()
                menu.show_emergency_stop()
                break

            # 2. Lectura de Sensores a través del HAL
            dist_front, dist_left, dist_right = hal.get_tof_distances()
            yaw = hal.get_yaw_deg()
            aruco_id = hal.get_aruco_id()

            # 3. Lógica de Navegación Reactiva y Seguimiento de Paredes
            if dist_front < FRONT_COLLISION_CM:
                # Obstáculo al frente: decidir giro según espacio lateral disponible
                fsm_state = "GIRO"
                if dist_left > dist_right:
                    # Girar a la izquierda sobre su propio eje
                    hal.set_motor_speeds(-BASE_SPEED_PWM, BASE_SPEED_PWM)
                else:
                    # Girar a la derecha sobre su propio eje
                    hal.set_motor_speeds(BASE_SPEED_PWM, -BASE_SPEED_PWM)
            else:
                # Avance con centrado proporcional
                fsm_state = "CENTRO"
                # Si ambas paredes están visibles dentro de rango razonable (< 35 cm)
                if dist_left < 35.0 and dist_right < 35.0:
                    error = dist_left - dist_right
                    steering = int(error * KP_WALL)
                    hal.set_motor_speeds(BASE_SPEED_PWM + steering, BASE_SPEED_PWM - steering)
                elif dist_left < 35.0:
                    # Siguiendo solo pared izquierda
                    error = dist_left - TARGET_WALL_DIST_CM
                    steering = int(error * KP_WALL)
                    hal.set_motor_speeds(BASE_SPEED_PWM + steering, BASE_SPEED_PWM - steering)
                elif dist_right < 35.0:
                    # Siguiendo solo pared derecha
                    error = TARGET_WALL_DIST_CM - dist_right
                    steering = int(error * KP_WALL)
                    hal.set_motor_speeds(BASE_SPEED_PWM + steering, BASE_SPEED_PWM - steering)
                else:
                    # Sin paredes cercanas, avance recto
                    hal.set_motor_speeds(BASE_SPEED_PWM, BASE_SPEED_PWM)

            # 4. Actualización de Telemetría en Pantalla OLED a 8 Hz
            now = time.time()
            if (now - last_telemetry_time) > 0.125:
                menu.show_telemetry("PISTA A", dist_front, dist_left, dist_right, yaw, aruco_id, fsm_state)
                last_telemetry_time = now

            time.sleep(0.02) # Ciclo de control a 50 Hz

    finally:
        hal.stop_all()


def run_track_b(hal, menu):
    """
    Rutina autónoma para Pista B (Recolección de pelotas y compuerta servo).
    """
    print("[MODO] === INICIANDO PISTA B ===")
    menu.show_countdown(3)
    
    BASE_SPEED_PWM = 150
    fsm_state = "BUSCA_PELOTA"
    last_telemetry_time = 0.0

    try:
        while True:
            # Parada de emergencia
            if menu.is_emergency_pressed():
                print("[EMERGENCIA] ¡Parada de emergencia en Pista B!")
                hal.stop_all()
                menu.show_emergency_stop()
                break

            dist_front, dist_left, dist_right = hal.get_tof_distances()
            yaw = hal.get_yaw_deg()

            # Rutina de exploración con compuerta cerrada
            if dist_front < 20.0:
                fsm_state = "EVITA_MURO"
                hal.set_motor_speeds(-BASE_SPEED_PWM, BASE_SPEED_PWM)
            else:
                fsm_state = "PATRULLAJE"
                hal.set_motor_speeds(BASE_SPEED_PWM, BASE_SPEED_PWM)

            # Refresco de telemetría en OLED
            now = time.time()
            if (now - last_telemetry_time) > 0.125:
                menu.show_telemetry("PISTA B", dist_front, dist_left, dist_right, yaw, None, fsm_state)
                last_telemetry_time = now

            time.sleep(0.02)

    finally:
        hal.stop_all()


# ==============================================================================
# BUCLE PRINCIPAL DEL SISTEMA (ENTRYPOINT)
# ==============================================================================

def main():
    print("==================================================")
    print("  CANDIDATES PRINCIPIANTES 2026 - ROBOT AUTÓNOMO  ")
    print("==================================================")

    # 1. Instanciar interfaz OLED
    menu = MenuManager()
    menu.show_splash("Iniciando HAL...")

    # 2. Instanciar Capa de Abstracción de Hardware
    hal = RobotHAL(enable_vision=True, enable_pixy=False)

    time.sleep(0.5)
    menu.draw_menu()

    try:
        while True:
            # Dibujar menú actual
            menu.draw_menu()

            # Esperar interacción de botones físicos
            btn = menu.check_buttons()

            if btn == 'NAV':
                # Avanzar a la siguiente opción cíclicamente
                menu.selected_index = (menu.selected_index + 1) % len(menu.menu_options)
                print(f"[MENU] Opción seleccionada: {menu.menu_options[menu.selected_index]}")

            elif btn == 'SEL':
                # Ejecutar la opción confirmada
                selected = menu.selected_index
                print(f"[MENU] Confirmado: {menu.menu_options[selected]}")

                if selected == 0:
                    run_track_a(hal, menu)
                elif selected == 1:
                    run_track_b(hal, menu)
                elif selected == 2:
                    run_sensor_diagnostic_mode(hal, menu)
                elif selected == 3:
                    run_calibrate_imu_mode(hal, menu)

                # Al terminar la rutina, asegurar motores apagados y redibujar menú
                hal.stop_all()
                time.sleep(0.4)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[SISTEMA] Interrupción por teclado recibida. Apagando...")

    finally:
        hal.stop_all()
        hal.close()
        print("[SISTEMA] Robot apagado de manera segura.")


if __name__ == '__main__':
    main()
