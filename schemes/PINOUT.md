# Mapeo Oficial de Pines (Pinout) - Orange Pi Zero 2W (Allwinner H618)

Este documento registra la asignación exacta de pines del conector de expansión de la **Orange Pi Zero 2W**, validada a partir del diagrama oficial del fabricante.

---

## 1. Tabla de Pines del Conector (Header)

| Pin Físico (Izq) | Nombre SoC | Función Primaria / Bus | | Pin Físico (Der) | Nombre SoC | Función Primaria / Bus |
| :---: | :---: | :--- | :---: | :---: | :---: | :--- |
| **1** | **3.3V** | Alimentación Lógica 3.3V | | **2** | **5V** | Alimentación Potencia 5.0V |
| **3** | `PI8` | **TWI1_SDA** (I2C-1 Datos) | | **4** | **GND** | Tierra / Masa |
| **5** | `PI7` | **TWI1_SCL** (I2C-1 Reloj) | | **6** | `PH0` | UART0_TX (Consola Debug) |
| **7** | `PI13` | PWM3 / UART4_TX | | **8** | `PH1` | UART0_RX (Consola Debug) |
| **9** | **GND** | Tierra / Masa | | **10** | `PI1` | GPIO General |
| **11** | `PH2` | **UART5_TX** (Salida hacia ESP32-C3-Zero RX) | | **12** | **GND** | Tierra / Masa |
| **13** | `PH3` | UART5_RX (No conectado - Unidireccional) | | **14** | `PI14` | PWM4 / UART4_RX |
| **15** | `PI5` | TWI0_SCL / UART2_TX | | **16** | `PH4` | GPIO General |
| **17** | **3.3V** | Alimentación Lógica 3.3V | | **18** | **GND** | Tierra / Masa |
| **19** | `PH7` | **SPI1_MOSI** (PixyCam v2) | | **20** | `PI6` | TWI0_SDA / UART2_RX |
| **21** | `PH8` | **SPI1_MISO** (PixyCam v2) | | **22** | `PH5` | **SPI1_CS0** (PixyCam v2 Chip Select) |
| **23** | `PH6` | **SPI1_CLK** (PixyCam v2 Reloj) | | **24** | `PH9` | SPI1_CS1 |
| **25** | **GND** | Tierra / Masa | | **26** | `PI9` | TWI2_SCL / UART3_TX |
| **27** | `PI10` | TWI2_SDA / UART3_RX | | **28** | **GND** | Tierra / Masa |
| **29** | `PI0` | GPIO General | | **30** | `PI11` | PWM1 |
| **31** | `PI15` | GPIO General | | **32** | **GND** | Tierra / Masa |
| **33** | `PI12` | PWM2 | | **34** | `PC12` | GPIO General |
| **35** | `PI2` | GPIO General | | **36** | `PI4` | GPIO General |
| **37** | `PI16` | GPIO General | | **38** | `PI3` | GPIO General |
| **39** | **GND** | Tierra / Masa | | - | - | - |

---

## 2. Asignación Oficial para el Robot Candidates 2026

### A. Bus I2C de Sensores y Pantalla (/dev/i2c-2, registro 5002400.i2c)
* **SDA:** Pin 3 (`PI8` / `TWI1_SDA`)
* **SCL:** Pin 5 (`PI7` / `TWI1_SCL`)
* **Dispositivos compartidos:**
  * VL53L1X Frontal: Dirección fija `0x29`
  * VL53L0X Lateral Izquierdo: Dirección reasignada `0x30`
  * VL53L0X Lateral Derecho: Dirección reasignada `0x31`
  * MPU-6050 (IMU): Dirección fija `0x68`
  * SSD1306 (OLED): Dirección fija `0x3C`

### B. Comunicación Serial UART5 (`/dev/ttyS5`) - Control ESP32-C3-Zero
* **TX:** Pin 11 (`PH2` / `UART5_TX`) -> Conectado a GPIO 20 (RX) del ESP32-C3-Zero.
* **GND:** Pin 9 o 12 -> Conectado a GND del ESP32-C3-Zero.
* **RX:** Pin 13 (`PH3` / `UART5_RX`) -> Desconectado (comunicación estrictamente unidireccional).

### C. Bus SPI1 (`/dev/spidev1.0`) - PixyCam v2
* **MOSI:** Pin 19 (`PH7`)
* **MISO:** Pin 21 (`PH8`)
* **CLK:** Pin 23 (`PH6`)
* **CS0:** Pin 22 (`PH5`)

### D. Líneas XSHUT para Secuencia de Arranque ToF (Salidas GPIO)
* **XSHUT Frontal:** Pin 16 (`PH4`)
* **XSHUT Izquierdo:** Pin 29 (`PI0`)
* **XSHUT Derecho:** Pin 31 (`PI15`)

### E. Pulsadores del Menú OLED (Entradas GPIO con Pull-Up)
* **Pulsador Navegar:** Pin 35 (`PI2`)
* **Pulsador Seleccionar:** Pin 37 (`PI16`)
