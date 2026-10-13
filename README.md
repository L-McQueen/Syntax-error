<div align="center">
  <h1>🤖 Enigma Seteki | Candidates 2026</h1>
  <p><strong>Robot móvil autónomo con arquitectura distribuida (Orange Pi Zero 2W + Arduino Uno R3), gemelo digital Sim2Real en Webots, navegación inercial con filtro complementario, visión híbrida y telemetría OLED en pits.</strong></p>
  
  <!-- Animated Typing SVG Header -->
  <a href="https://github.com/L-McQueen/Syntax-error">
    <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&weight=600&size=20&pause=1000&color=38BDF8&center=true&vCenter=true&width=550&lines=3nIgMa+S3t3ki+%7C+Candidates+2026;Arquitectura+Distribuida+SBC+%2B+MCU;Gemelo+Digital+Sim2Real+en+Webots;Filtro+Complementario+Inercial;Control+Lateral+PID+en+Pasillos;Cero+Cables+en+Pista" alt="Typing SVG" />
  </a>

  <p>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/Platform-Orange_Pi_Zero_2W-orange.svg?style=for-the-badge&logo=linux" alt="Orange Pi"></a>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/MCU-Arduino_Uno_R3-00979D.svg?style=for-the-badge&logo=arduino" alt="Arduino"></a>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/Simulation-Webots_R2023b-00599C.svg?style=for-the-badge" alt="Webots"></a>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/Driver-DRV8833_Dual_MOSFET-lightgrey.svg?style=for-the-badge" alt="DRV8833"></a>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/Sensors-ToF_VL53L1X_%2B_VL53L0X-yellow.svg?style=for-the-badge" alt="Sensors"></a>
    <a href="https://github.com/L-McQueen/Syntax-error"><img src="https://img.shields.io/badge/Status-Field_Ready-brightgreen.svg?style=for-the-badge" alt="Status"></a>
  </p>
</div>

---

> [!IMPORTANT]
> **Repositorio Oficial de Ingeniería:** Este proyecto documenta el diseño, modelado físico, circuitería y control autónomo de **3nIgMa S3t3ki** para la categoría **Candidates Principiantes 2026**. Todo el desarrollo sigue una rigurosa metodología *Sim2Real*, validando algoritmos en gemelo digital antes de implementarlos en hardware real.

> [!NOTE]
> El robot opera con autonomía total en competencia: no requiere enlaces por Wi-Fi, pantallas externas ni laptops en pista. Toda la selección de modos, calibración inercial y telemetría en vivo se gestiona a través de la pantalla OLED SSD1306 y los pulsadores del chasis.

---

## 🗂️ Estructura y Navegación del Repositorio

```
📦 Candidates-2026 (3nIgMa S3t3ki)
├── 📁 3D-models/             # Archivo CAD ensamble.step, cinemática y parámetros de impresión 3D
│   ├── 📄 ensamble.step      # Modelo tridimensional maestro AP214
│   └── 📄 README.md          # Especificaciones mecánicas, masas y tolerancias de manufactura
│
├── 📁 components/            # Catálogo técnico de los 10 componentes del robot
│   └── 📄 README.md          # Datasheets, consumo, pinouts y justificación técnica
│
├── 📁 schemes/               # Arquitectura eléctrica, esquemáticos y pines
│   ├── 📄 PINOUT.md          # Mapeo oficial de los 26 pines de la Orange Pi Zero 2W
│   └── 📄 README.md          # Diagramas de cableado, rieles aislados de 7.8V/5.0V y buses
│
├── 📁 robot-logic/           # Algoritmos, matemáticas y control en tiempo real
│   └── 📄 README.md          # Filtro complementario en Y, centrado PID, FSMs y Watchdog
│
├── 📁 src/                   # Código fuente completo del proyecto
│   ├── 📁 arduino/           # Firmware C++ con Watchdog de tracción y PWM determinista
│   ├── 📁 orange/            # Orquestador Python, menú OLED, HAL multihilo y suite de tests
│   ├── 📁 webots-sim2real/   # Gemelo digital en Webots (mundos 3D, controllers y supervisors)
│   └── 📄 README.md          # Guía general de arquitectura y flujo de software
│
├── 📁 media/                 # Registro audiovisual del robot
│   ├── 📁 robot-photos/      # Galería fotográfica del chasis y electrónica en alta resolución
│   ├── 📁 videos/            # Demostraciones de navegación y pruebas en pista
│   └── 📄 README.md          # Galería interactiva con marcos y enlaces a YouTube
│
└── 📄 reglamento_candidates_principiantes_2026.md # Reglamento oficial de la competencia
```

---

## 📌 1. Visión General y Filosofía de Ingeniería

El reto de **Candidates 2026** exige que el robot resuelva dos escenarios complejos de manera 100% autónoma:
1. **Pista A (MAZE):** Un laberinto ortogonal de 5x5 celdas ($1.50\text{ m} \times 1.50\text{ m}$), con pasillos estrechos de 30 cm, baldosas de colores reglamentarios y callejones sin salida.
2. **Pista B (Niveles y Desniveles):** Un circuito con rampas inclinadas, cambios de elevación y recolección/descarga de pelotas.

### Metodología de Diseño:
* **Gemelo Digital Sim2Real:** Modelado honesto en Webots R2023b que incluye micro-patinaje de neumáticos, asimetría de tracción entre motores N20, deriva térmica estocástica en el giróscopo MPU-6050 y cono de dispersión en los sensores láser ToF.
* **Arquitectura Jerárquica Desacoplada:** El cálculo pesado (visión, navegación en grafos, interfaz de usuario) se confía al procesador ARM64 de la SBC, mientras la generación de PWM de alta fidelidad y la seguridad motriz se delegan al microcontrolador dedicado.
* **Aislamiento Galvánico:** Desacoplamiento total entre la línea de potencia de los motores ($7.8\text{ V}$) y la línea lógica ($5.0\text{ V}$) para eliminar reinicios repentinos (*brownouts*) por picos inductivos.

---

## 🆔 2. Identidad del Robot: ¿Por qué 3nIgMa?

El nombre **3nIgMa S3t3ki** rinde homenaje a dos conceptos fundamentales:
1. **Enigma (Criptografía y Lógica):** Al igual que la legendaria máquina de descifrado, el robot debe descifrar y reconstruir en tiempo real la topología desconocida del laberinto ortogonal, interpretando señales sensoriales ruidosas y tomando decisiones de navegación deterministas.
2. **S3t3ki (Inspiración en Resiliencia):** Representa la perseverancia de la ingeniería aplicada frente a las limitaciones de presupuesto, maximizando la física y las matemáticas para lograr resultados de clase mundial con componentes accesibles.

---

## 🛠️ 3. Arquitectura Eléctrica y Setup Físico

```
                     ┌────────────────────────────────────────┐
                     │          Batería LiPo 3S (11.1V)       │
                     └───────────────────┬────────────────────┘
                                         │
                         ┌───────────────┴───────────────┐
                         ▼                               ▼
               [LÍNEA DE POTENCIA]               [LÍNEA LÓGICA]
                Buck LM2596 (7.8V)             Regulador 5.0V Estable
                         │                               │
                         │               ┌───────────────┴───────────────┐
                         │               ▼                               ▼
                         │        Arduino Uno R3 ◄───UART (TX->RX)─── Orange Pi Zero 2W
                         │       (Tracción / PWM)  (115200 Baud Simplex) (Linux ARM64 / HAL)
                         │               │                               │
                         ▼               ▼                               │
                   Driver DRV8833 ◄──────┘ (D3, D5, D6, D11)             │
                   (Puente H MOSFET)                                     │
                         │                                               │
                         ▼                                               ▼
                  Motores N20 (100:1)                           Periféricos I2C y SPI:
                   (Tracción 7.8V)                               - Bus /dev/i2c-2:
                                                                   * IMU MPU-6050 (0x68)
                                                                   * OLED SSD1306 (0x3C)
                                                                   * ToF Frontal VL53L1X (0x29)
                                                                   * ToFs Laterales VL53L0X (0x30/31)
                                                                 - Bus SPI /dev/spidev1.0:
                                                                   * PixyCam v2 (Color de suelo)
                                                                 - Puerto USB /dev/video1:
                                                                   * Webcam USB (Visión ArUco)
```

---

## ⚙️ 4. Resumen de Componentes Clave

| Subsistema | Componente | Conexión / Bus | Función Principal |
| :--- | :--- | :--- | :--- |
| **Cerebro Superior** | [Orange Pi Zero 2W](components/) | Linux Debian 12 | Fusión sensorial, navegación de laberinto, visión y menú OLED |
| **Controlador PWM** | [Arduino Uno R3](components/) | UART `/dev/ttyS5` (115200) | Generación determinista de PWM y Watchdog de seguridad (300 ms) |
| **Driver de Motores** | [DRV8833 Dual MOSFET](components/) | PWM D3, D5, D6, D11 | Tracción eficiente con caída interna mínima ($\approx 0.1\text{ V}$) |
| **Actuadores** | [Motores N20 (100:1)](components/) | Tracción 7.8V fija | Reducción metálica de alto par para ascender rampas |
| **Distancia Frontal** | [Pololu VL53L1X](components/) | `/dev/i2c-2` (0x29) | Detección de paredes a largo alcance (hasta 4.0 m) |
| **Distancia Lateral** | [2x Pololu VL53L0X](components/) | `/dev/i2c-2` (0x30 y 0x31) | Centrado lateral PID en pasillos de 30 cm |
| **Sensor Inercial** | [GY-521 MPU-6050](components/) | `/dev/i2c-2` (0x68) | Rumbo horizontal (Yaw en Z) y cabeceo de rampa (Pitch en Y) |
| **Telemetría en Pits** | [SSD1306 OLED 0.96"](components/) | `/dev/i2c-2` (0x3C) | Interfaz visual y diagnóstico sin cables |

---

## 🧠 5. Control y Algoritmos en Tiempo Real

### A. Filtro Complementario en Eje Y (Detección de Rampas)
El sensor MPU-6050 tiene su eje Y montado en la dirección de avance:
```python
pitch_acc = math.atan2(Ay, math.sqrt(Ax**2 + Az**2)) * (180.0 / math.pi)
pitch = 0.96 * (pitch + gx_corr * dt) + 0.04 * pitch_acc
```
* `pitch > +6.0°`: Estado **SUBIDA** (aumenta potencia a motores).
* `pitch < -6.0°`: Estado **BAJADA** (frenado dinámico por contramarcha).
* `-6.0° <= pitch <= +6.0°`: Estado **PLANO** (navegación crucero normal).

### B. Centrado Lateral PID en Laberinto
En pasillos de 30 cm, el error de centrado $e(t) = d_{\text{izq}} - d_{\text{der}}$ modula la velocidad de las ruedas:
$$PWM_{\text{izq}} = PWM_{\text{base}} - (K_p \cdot e(t) + K_d \cdot \dot{e}(t))$$
$$PWM_{\text{der}} = PWM_{\text{base}} + (K_p \cdot e(t) + K_d \cdot \dot{e}(t))$$

### C. Failsafe Hardware (Watchdog de 300 ms en Arduino)
Si el software en Linux se bloquea o el cable serial sufre desconexión, el microcontrolador corta la energía a los motores en **menos de 300 ms**, evitando colisiones mecánicas accidentales.

---

## 🖥️ 6. Interfaz OLED SSD1306 y Telemetría en Vivo

```
+--------------------------------+
| IMU EN VIVO           31 C     | <- Barra superior invertida con temperatura
| YAW    : 120.5*   [GIRO DER]   | <- Rumbo acumulado e indicador de viraje
| CABECEO: +14.2*   [SUBIDA ^]   | <- Detección dinámica de rampa
| ACEL   : Ax:+0.1 Ay:+2.4 Az:+9.6|<- Aceleraciones instantáneas en m/s^2
| [=== Indicador Dinámico ===]   | <- Barra gráfica horizontal centrada
+--------------------------------+
```

* **Pulsador Navegar (Pin 35 / GPIO):** Cambia de opción en el menú.
* **Pulsador Seleccionar (Pin 37 / GPIO):** Confirma la pista e inicia cuenta regresiva de 3 segundos (`3... 2... 1... !VAMOOOS!`).
* **Parada de Emergencia:** Cualquier pulsación física en carrera detiene los motores en seco en menos de 20 ms.

---

## 🚀 7. Guía de Puesta en Marcha Rápida

### 1. Flashear el Arduino Uno
1. Abre el sketch `src/arduino/firmware_motor_controller/firmware_motor_controller.ino` en el Arduino IDE.
2. Selecciona la placa **Arduino Uno** y sube el firmware.

### 2. Sincronizar el Software con la Orange Pi
Desde tu terminal en Windows:
```cmd
./src/orange/sync.bat
```

### 3. Ejecutar Pruebas Modulares de Diagnóstico
Conéctate por SSH a la Orange Pi y corre la suite de pruebas unitarias:
```bash
# Probar pantalla OLED SSD1306
python3 ~/orange/tests/test_screen.py

# Probar sensor inercial MPU-6050
python3 ~/orange/tests/test_imu.py

# Probar integración combinada IMU + pantalla OLED
python3 ~/orange/tests/test_imu_screen.py
```

### 4. Lanzar Modo Autónomo de Competencia
Inicia el orquestador oficial:
```bash
python3 ~/orange/main.py
```
Selecciona con los botones físicos entre **Pista A**, **Pista B** o **Diagnóstico en Pits**.

---

<div align="center">
  <p><strong>3nIgMa S3t3ki</strong> — <i>Diseñado en gemelo digital, construido en hardware real, validado en pista.</i></p>
</div>
