# Mapeo Oficial de Pines (Pinout) - Orange Pi Zero 2W (Allwinner H618)

Este documento registra la asignación exacta de pines del conector de expansión de la **Orange Pi Zero 2W**, validada a partir del diagrama oficial del fabricante (*User Manual v1.3, Pág. 141*), el código fuente del driver `wiringOP` y la imagen esquemática [pinout.jpeg](../pinout.jpeg).

---

## 1. Tabla de Pines del Conector (Header CON2 de 40 Pines)

| Pin Físico (Izq) | GPIO NO. | Nombre SoC | Función Primaria / Bus | | Pin Físico (Der) | GPIO NO. | Nombre SoC | Función Primaria / Bus |
| :---: | :---: | :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **1** | - | **3.3V** | Alimentación Lógica 3.3V | | **2** | - | **5V** | Alimentación Potencia 5.0V (VBUS) |
| **3** | `264` | `PI8` | **TWI1_SDA** (I2C-1 Datos) | | **4** | - | **5V** | Segundo Riel Potencia 5.0V |
| **5** | `263` | `PI7` | **TWI1_SCL** (I2C-1 Reloj) | | **6** | - | **GND** | Tierra / Masa Común |
| **7** | `269` | `PI13` | PWM3 / UART4_TX | | **8** | `224` | `PH0` | UART0_TX (Consola Debug) |
| **9** | - | **GND** | Tierra / Masa Común | | **10** | `225` | `PH1` | UART0_RX (Consola Debug) |
| **11** | `226` | `PH2` | **UART5_TX** (Salida -> ESP32-C3 RX) | | **12** | `257` | `PI1` | GPIO General |
| **13** | `227` | `PH3` | UART5_RX (Desconectado - Simplex) | | **14** | - | **GND** | Tierra / Masa Común |
| **15** | `261` | `PI5` | TWI0_SCL / UART2_TX | | **16** | `270` | `PI14` | PWM4 / UART4_RX |
| **17** | - | **3.3V** | Alimentación Lógica 3.3V | | **18** | `228` | `PH4` | GPIO General (**XSHUT Frontal**) |
| **19** | `231` | `PH7` | **SPI1_MOSI** (PixyCam v2) | | **20** | - | **GND** | Tierra / Masa Común |
| **21** | `232` | `PH8` | **SPI1_MISO** (PixyCam v2) | | **22** | `262` | `PI6` | TWI0_SDA / UART2_RX |
| **23** | `230` | `PH6` | **SPI1_CLK** (PixyCam v2 Reloj) | | **24** | `229` | `PH5` | **SPI1_CS0** (PixyCam v2 Chip Select) |
| **25** | - | **GND** | Tierra / Masa Común | | **26** | `233` | `PH9` | SPI1_CS1 |
| **27** | `266` | `PI10` | TWI2_SDA / UART3_RX | | **28** | `265` | `PI9` | TWI2_SCL / UART3_TX |
| **29** | `256` | `PI0` | GPIO General (**XSHUT Izquierdo**) | | **30** | - | **GND** | Tierra / Masa Común |
| **31** | `271` | `PI15` | GPIO General (**XSHUT Derecho**) | | **32** | `267` | `PI11` | **PWM1** (PWM Hardware) |
| **33** | `268` | `PI12` | **PWM2** (PWM Hardware) | | **34** | - | **GND** | Tierra / Masa Común |
| **35** | `258` | `PI2` | GPIO General (**Pulsador Navegar**) | | **36** | `76` | `PC12` | GPIO General |
| **37** | `272` | `PI16` | GPIO General (**Pulsador Seleccionar**) | | **38** | `260` | `PI4` | GPIO General |
| **39** | - | **GND** | Tierra / Masa Común | | **40** | `259` | `PI3` | GPIO General |

---

## 2. Asignación Oficial para el Robot Candidates 2026

### A. Bus I2C de Sensores y Pantalla (/dev/i2c-2, registro 5002400.i2c)
* **SDA:** Pin 3 (`PI8` / `TWI1_SDA` / GPIO 264)
* **SCL:** Pin 5 (`PI7` / `TWI1_SCL` / GPIO 263)
* **Dispositivos compartidos:**
  * VL53L1X Frontal: Dirección fija `0x29`
  * VL53L0X Lateral Izquierdo: Dirección reasignada `0x30`
  * VL53L0X Lateral Derecho: Dirección reasignada `0x31`
  * MPU-6050 (IMU): Dirección fija `0x68`
  * SSD1306 (OLED): Dirección fija `0x3C`

### B. Comunicación Serial UART5 (`/dev/ttyS5`) - Control ESP32-C3-Zero
* **TX:** Pin 11 (`PH2` / `UART5_TX` / GPIO 226) -> Conectado a GPIO 20 (RX) del ESP32-C3-Zero.
* **GND:** Pin 6, 9, 14, 20, 25, 30 o 34 -> Conectado a GND común del ESP32-C3-Zero.
* **RX:** Pin 13 (`PH3` / `UART5_RX` / GPIO 227) -> Desconectado (comunicación estrictamente unidireccional).

### C. Bus SPI1 (`/dev/spidev1.0`) - PixyCam v2
* **MOSI:** Pin 19 (`PH7` / GPIO 231)
* **MISO:** Pin 21 (`PH8` / GPIO 232)
* **CLK:** Pin 23 (`PH6` / GPIO 230)
* **CS0:** Pin 24 (`PH5` / GPIO 229)

### D. Líneas XSHUT para Secuencia de Arranque ToF (Salidas GPIO)
* **XSHUT Frontal:** Pin 18 (`PH4` / GPIO 228)
* **XSHUT Izquierdo:** Pin 29 (`PI0` / GPIO 256)
* **XSHUT Derecho:** Pin 31 (`PI15` / GPIO 271)

### E. Pulsadores del Menú OLED (Entradas GPIO con Pull-Up)
* **Pulsador Navegar:** Pin 35 (`PI2` / GPIO 258)
* **Pulsador Seleccionar:** Pin 37 (`PI16` / GPIO 272)
