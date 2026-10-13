"""Test individual de pantalla OLED SSD1306 (128x64 I2C /dev/i2c-2 en 0x3C)."""

import os
import sys
import time

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont


# --- CONFIGURACIÓN DE LA PANTALLA ---
# /dev/i2c-2 es el bus físico de los pines 3 (SDA) y 5 (SCL)
I2C_PORT = 2
I2C_ADDRESS = 0x3C
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64


def main():
    print("=" * 65)
    print("   TEST INDIVIDUAL: PANTALLA OLED SSD1306 (I2C-2 0x3C)   ")
    print("=" * 65)

    print(f"[INFO] Conectando a SSD1306 en /dev/i2c-{I2C_PORT} (0x{I2C_ADDRESS:02X})...")

    device = None
    try:
        serial_i2c = i2c(port=I2C_PORT, address=I2C_ADDRESS)
        device = ssd1306(serial_i2c, width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
        print("[OK] ¡Pantalla OLED inicializada correctamente!")
    except Exception as e:
        print(f"\n[ERROR CRÍTICO] No se pudo abrir la pantalla en /dev/i2c-{I2C_PORT}:")
        print(f"Detalle: {e}")
        print("\nVerificaciones recomendadas:")
        print(" 1. ¿VCC está en Pin 2 (5V) o Pin 1 (3.3V) y GND en tierra?")
        print(" 2. ¿SDA está en Pin 3 y SCL en Pin 5?")
        print(" 3. Ejecuta: i2cdetect -y 2")
        print("    Debería aparecer el número '3c' en la matriz.")
        sys.exit(1)

    font_default = ImageFont.load_default()

    try:
        # ----------------------------------------------------------------------
        # TEST 1: Pantalla de Bienvenida y Marco Exterior
        # ----------------------------------------------------------------------
        print("\n[1/4] Mostrando pantalla de presentación...")
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)

        # Marco exterior con esquinas
        draw.rectangle((0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1), outline=1)
        # Encabezado invertido (fondo blanco, texto negro)
        draw.rectangle((2, 2, SCREEN_WIDTH - 3, 14), fill=1)
        draw.text((10, 3), "CANDIDATES 2026", font=font_default, fill=0)

        draw.text((12, 24), "TEST DE PANTALLA", font=font_default, fill=1)
        draw.text((22, 40), "OLED SSD1306", font=font_default, fill=1)
        draw.text((36, 52), "[I2C-2 OK]", font=font_default, fill=1)

        device.display(img)
        time.sleep(2.0)

        # ----------------------------------------------------------------------
        # TEST 2: Prueba Geométrica de Píxeles (Cuadrantes y Círculos)
        # ----------------------------------------------------------------------
        print("[2/4] Probando píxeles y formas geométricas...")
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)

        # Líneas diagonales de esquina a esquina
        draw.line((0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1), fill=1)
        draw.line((0, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, 0), fill=1)

        # Círculos concéntricos en el centro
        draw.ellipse((44, 12, 84, 52), outline=1, fill=0)
        draw.ellipse((52, 20, 76, 44), fill=1)

        # Recuadros en las 4 esquinas
        draw.rectangle((2, 2, 16, 16), outline=1)
        draw.rectangle((SCREEN_WIDTH - 17, 2, SCREEN_WIDTH - 3, 16), outline=1)
        draw.rectangle((2, SCREEN_HEIGHT - 17, 16, SCREEN_HEIGHT - 3), outline=1)
        draw.rectangle((SCREEN_WIDTH - 17, SCREEN_HEIGHT - 17, SCREEN_WIDTH - 3, SCREEN_HEIGHT - 3), outline=1)

        device.display(img)
        time.sleep(2.0)

        # ----------------------------------------------------------------------
        # TEST 3: Animación de Barra de Progreso Fluida (Refresco y FPS)
        # ----------------------------------------------------------------------
        print("[3/4] Ejecutando animación de barra de progreso...")
        total_steps = 100
        for step in range(total_steps + 1):
            img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
            draw = ImageDraw.Draw(img)

            draw.text((16, 8), "CARGANDO SISTEMA...", font=font_default, fill=1)

            # Contenedor de la barra de progreso
            bar_x1, bar_y1 = 14, 26
            bar_x2, bar_y2 = 114, 40
            draw.rectangle((bar_x1, bar_y1, bar_x2, bar_y2), outline=1)

            # Relleno proporcional
            fill_width = int((bar_x2 - bar_x1 - 4) * (step / total_steps))
            if fill_width > 0:
                draw.rectangle((bar_x1 + 2, bar_y1 + 2, bar_x1 + 2 + fill_width, bar_y2 - 2), fill=1)

            draw.text((48, 48), f"{step} %", font=font_default, fill=1)
            device.display(img)
            time.sleep(0.02) # ~50 FPS

        time.sleep(0.5)

        # ----------------------------------------------------------------------
        # TEST 4: Simulación de Telemetría en Vivo de Competencia
        # ----------------------------------------------------------------------
        print("[4/4] Simulando telemetría de carrera en vivo (5 segundos)...")
        start_time = time.time()
        step_sim = 0

        while (time.time() - start_time) < 5.0:
            step_sim += 1
            elapsed = time.time() - start_time

            # Datos simulados con oscilación suave
            sim_yaw = (elapsed * 35.0) % 360.0
            sim_front = 25.0 + 10.0 * (step_sim % 10) / 10.0
            sim_left = 14.5 + (step_sim % 5) * 0.2
            sim_right = 15.2 - (step_sim % 5) * 0.2
            sim_aruco = 12 if int(elapsed) % 2 == 0 else "---"

            img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
            draw = ImageDraw.Draw(img)

            # Barra superior
            draw.rectangle((0, 0, SCREEN_WIDTH, 11), fill=1)
            draw.text((4, 1), "PISTA A | NAVEGANDO", font=font_default, fill=0)

            # Lecturas ToF
            draw.text((2, 15), f"L:{sim_left:4.1f} F:{sim_front:4.1f} R:{sim_right:4.1f}", font=font_default, fill=1)

            # Rumbo e ID ArUco
            draw.text((2, 30), f"Yaw:{sim_yaw:5.1f}*  Ar:{sim_aruco}", font=font_default, fill=1)

            # Barra inferior con aviso
            draw.line((0, 46, SCREEN_WIDTH, 46), fill=1)
            draw.text((10, 50), f"T:{elapsed:4.1f}s [OK RUNNING]", font=font_default, fill=1)

            device.display(img)
            time.sleep(0.08) # ~12 Hz de refresco

        # ----------------------------------------------------------------------
        # FINAL: Pantalla de Éxito
        # ----------------------------------------------------------------------
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)

        draw.rectangle((4, 4, SCREEN_WIDTH - 5, SCREEN_HEIGHT - 5), outline=1)
        draw.rectangle((8, 8, SCREEN_WIDTH - 9, SCREEN_HEIGHT - 9), outline=1)
        draw.text((20, 18), "!PANTALLA OK!", font=font_default, fill=1)
        draw.text((14, 34), "TODOS LOS PIXELES", font=font_default, fill=1)
        draw.text((22, 48), "FUNCIONANDO :D", font=font_default, fill=1)

        device.display(img)
        print("\n[LISTO] Prueba completada con éxito. La pantalla responde a la perfección.")

    except KeyboardInterrupt:
        print("\n[INFO] Prueba interrumpida por el usuario.")
    finally:
        # Pequeña pausa antes de salir
        time.sleep(1.0)


if __name__ == '__main__':
    main()
