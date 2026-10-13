"""Menú interactivo en pantalla OLED SSD1306 y lectura de pulsadores GPIO."""

import os
import sys
import time

import gpiod
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# Configuración de hardware
OLED_I2C_PORT = 2
OLED_I2C_ADDRESS = 0x3C
SCREEN_WIDTH = 128
SCREEN_HEIGHT = 64

GPIO_CHIP_NAME = "/dev/gpiochip0"
PIN_BTN_NAVIGATE = 258  # PI2 (Pin 35)
PIN_BTN_SELECT   = 272  # PI16 (Pin 37)
DEBOUNCE_DELAY_SEC = 0.18


class MenuManager:
    def __init__(self):
        self.device = None
        self.font_small = None
        self.font_large = None

        self.chip = None
        self.btn_nav_line = None
        self.btn_sel_line = None
        self.last_press_time = 0.0

        self.menu_options = [
            "1. PISTA A (LABERINTO)",
            "2. PISTA B (PELOTAS)",
            "3. TEST SENSORES",
            "4. CALIBRAR IMU"
        ]
        self.selected_index = 0

        self._init_display()
        self._init_buttons()

    def _init_display(self):
        try:
            serial_i2c = i2c(port=OLED_I2C_PORT, address=OLED_I2C_ADDRESS)
            self.device = ssd1306(serial_i2c, width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
            self.font_small = ImageFont.load_default()
            self.font_large = ImageFont.load_default()
        except Exception as e:
            print(f"[ERROR] No se pudo inicializar OLED: {e}")

    def _init_buttons(self):
        try:
            if os.path.exists(GPIO_CHIP_NAME):
                self.chip = gpiod.Chip(GPIO_CHIP_NAME)
                self.btn_nav_line = self.chip.get_line(PIN_BTN_NAVIGATE)
                self.btn_nav_line.request(consumer="btn_nav", type=gpiod.LINE_REQ_DIR_IN)
                self.btn_sel_line = self.chip.get_line(PIN_BTN_SELECT)
                self.btn_sel_line.request(consumer="btn_sel", type=gpiod.LINE_REQ_DIR_IN)
        except Exception as e:
            print(f"[ERROR] No se pudieron configurar pulsadores GPIO: {e}")

    def check_buttons(self):
        """Retorna 'NAV', 'SEL' o None según los pulsadores físicos."""
        now = time.time()
        if (now - self.last_press_time) < DEBOUNCE_DELAY_SEC:
            return None

        if self.btn_nav_line:
            try:
                if self.btn_nav_line.get_value() == 0:
                    self.last_press_time = now
                    return 'NAV'
            except Exception:
                pass

        if self.btn_sel_line:
            try:
                if self.btn_sel_line.get_value() == 0:
                    self.last_press_time = now
                    return 'SEL'
            except Exception:
                pass

        return None

    def is_emergency_pressed(self):
        """Verifica si se presionó cualquier botón para parada de emergencia."""
        if self.btn_nav_line:
            try:
                if self.btn_nav_line.get_value() == 0:
                    return True
            except Exception:
                pass

        if self.btn_sel_line:
            try:
                if self.btn_sel_line.get_value() == 0:
                    return True
            except Exception:
                pass

        return False

    def show_splash(self, message="Iniciando robot..."):
        if not self.device:
            return
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1), outline=1)
        draw.rectangle((2, 2, SCREEN_WIDTH - 3, 14), fill=1)
        draw.text((8, 3), "CANDIDATES 2026", font=self.font_small, fill=0)
        draw.text((10, 26), "ROBOT PRINCIP.", font=self.font_small, fill=1)
        draw.text((10, 42), message[:20], font=self.font_small, fill=1)
        self.device.display(img)

    def draw_menu(self):
        if not self.device:
            return
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, SCREEN_WIDTH, 12), fill=1)
        draw.text((12, 1), "SELECCIONA MODO", font=self.font_small, fill=0)

        y_offset = 15
        for i, opt in enumerate(self.menu_options):
            if i == self.selected_index:
                draw.rectangle((2, y_offset - 1, SCREEN_WIDTH - 3, y_offset + 10), fill=1)
                draw.text((4, y_offset), f"> {opt}", font=self.font_small, fill=0)
            else:
                draw.text((8, y_offset), opt, font=self.font_small, fill=1)
            y_offset += 12

        self.device.display(img)

    def show_countdown(self, seconds=3):
        for sec in range(seconds, 0, -1):
            if self.device:
                img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
                draw = ImageDraw.Draw(img)
                draw.text((20, 4), "PREPARATE...", font=self.font_small, fill=1)
                draw.rectangle((44, 18, 84, 52), outline=1)
                draw.text((58, 28), str(sec), font=self.font_large, fill=1)
                progress_w = int((SCREEN_WIDTH - 8) * (1.0 - (sec / seconds)))
                draw.rectangle((4, 56, 4 + progress_w, 61), fill=1)
                self.device.display(img)
            time.sleep(1.0)

        if self.device:
            img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=1)
            draw = ImageDraw.Draw(img)
            draw.text((25, 26), "!VAMOOOS!", font=self.font_small, fill=0)
            self.device.display(img)
            time.sleep(0.4)

    def show_telemetry(self, mode_name, dist_f, dist_l, dist_r, yaw_deg, aruco_id=None, fsm_state="RUN"):
        if not self.device:
            return
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, SCREEN_WIDTH, 11), fill=1)
        draw.text((4, 1), f"{mode_name} | {fsm_state[:6]}", font=self.font_small, fill=0)
        draw.text((2, 14), f"L:{dist_l:3.0f} F:{dist_f:3.0f} R:{dist_r:3.0f}", font=self.font_small, fill=1)
        aruco_str = f"#{aruco_id}" if aruco_id is not None else "---"
        draw.text((2, 28), f"Yaw:{yaw_deg:5.1f}* Ar:{aruco_str}", font=self.font_small, fill=1)
        draw.line((0, 44, SCREEN_WIDTH, 44), fill=1)
        draw.text((8, 48), "[CUALQ. BTN = STOP]", font=self.font_small, fill=1)
        self.device.display(img)

    def show_sensor_diagnostic(self, hal):
        if not self.device:
            return
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=0)
        draw = ImageDraw.Draw(img)
        df, dl, dr = hal.get_tof_distances()
        yaw = hal.get_yaw_deg()
        pitch = hal.get_pitch_deg()
        aruco = hal.get_aruco_id()

        draw.rectangle((0, 0, SCREEN_WIDTH, 11), fill=1)
        draw.text((4, 1), "DIAGNOSTICO PITS", font=self.font_small, fill=0)
        draw.text((2, 14), f"Fte:{df:4.1f} Izq:{dl:4.1f} Der:{dr:4.1f}", font=self.font_small, fill=1)
        draw.text((2, 26), f"Yaw:{yaw:5.1f}* Pit:{pitch:+5.1f}*", font=self.font_small, fill=1)
        ar_str = str(aruco) if aruco is not None else "Ninguno"
        draw.text((2, 38), f"ArUco: {ar_str}", font=self.font_small, fill=1)
        draw.line((0, 50, SCREEN_WIDTH, 50), fill=1)
        draw.text((8, 52), "[BTN = SALIR]", font=self.font_small, fill=1)
        self.device.display(img)

    def show_emergency_stop(self):
        if not self.device:
            return
        img = Image.new("1", (SCREEN_WIDTH, SCREEN_HEIGHT), color=1)
        draw = ImageDraw.Draw(img)
        draw.rectangle((4, 4, SCREEN_WIDTH - 5, SCREEN_HEIGHT - 5), fill=0)
        draw.text((12, 16), "!PARADA EMERGENCIA!", font=self.font_small, fill=1)
        draw.text((18, 34), "Motores Apagados", font=self.font_small, fill=1)
        self.device.display(img)
        time.sleep(1.5)
