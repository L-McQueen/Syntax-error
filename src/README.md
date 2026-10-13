# 💻 Arquitectura de Software y Código Fuente (`src/`)

<div align="center">
  <p><strong>Diseño de software desacoplado: Computación de alto nivel en Linux ARM64, control de tiempo real en C++ y gemelo digital Sim2Real en Webots.</strong></p>
  <p>
    <img src="https://img.shields.io/badge/Language-Python_3.10%2B-blue.svg" alt="Python">
    <img src="https://img.shields.io/badge/Firmware-C%2B%2B_Arduino-00979D.svg" alt="Arduino C++">
    <img src="https://img.shields.io/badge/Digital_Twin-Webots_R2023b-red.svg" alt="Webots">
    <img src="https://img.shields.io/badge/Architecture-Decoupled_HAL-green.svg" alt="Architecture">
  </p>
</div>

---

> [!NOTE]
> La suite de código está organizada en tres subsistemas completamente independientes para permitir pruebas modulares aisladas antes de la integración total en campo.

---

## 📂 Organización de Carpetas en `src/`

```
📦 src
├── 📁 arduino/                       # Control de tracción y PWM determinista (C++)
│   └── 📁 firmware_motor_controller/
│       └── 📄 firmware_motor_controller.ino # Sketch Arduino con Watchdog de 300 ms
│
├── 📁 orange/                        # Software de alto nivel en Linux (Python)
│   ├── 📄 main.py                    # Orquestador autónomo de competencia y FSM
│   ├── 📄 menu.py                    # Interfaz gráfica OLED (SSD1306) y botones GPIO
│   ├── 📄 robot_hal.py               # Capa de Abstracción de Hardware (HAL) multihilo
│   ├── 📄 sync.bat / sync.ps1        # Scripts de sincronización rápida por SCP/SSH
│   └── 📁 tests/                     # Suite de pruebas unitarias de hardware
│       ├── 📄 test_screen.py         # Test de renderizado en pantalla OLED
│       ├── 📄 test_imu.py            # Test y calibración de 6 ejes MPU-6050
│       └── 📄 test_imu_screen.py     # Integración conjunta IMU + OLED en tiempo real
│
└── 📁 webots-sim2real/               # Gemelo digital de competencia en Webots
    ├── 📁 controllers/               # Controladores Python de simulación y supervisores
    ├── 📁 worlds/                    # Mundos virtuales 3D de Pista A (Maze) y Pista B (Niveles)
    ├── 📁 scripts/                   # Scripts auxiliares de generación procedural
    ├── 📁 docs/                      # Documentación del gemelo digital
    ├── 📄 launch.bat                 # Lanzador en un clic para Webots
    └── 📄 README.md                  # Documentación profunda de la simulación
```

---

## 🔄 Flujo de Datos e Interacción entre Módulos

```
 ┌────────────────────────────────────────────────────────────────────────┐
 │                         GEMELO DIGITAL (Webots)                        │
 │  * Simulación con ruido estocástico de sensores ToF y deriva inercial  │
 │  * Validación de la FSM de Pista A y Pista B antes de tocar hardware   │
 └───────────────────────────────────┬────────────────────────────────────┘
                                     │ Transferencia de Código Sim2Real
                                     ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                      ORANGE PI ZERO 2W (src/orange/)                   │
 │                                                                        │
 │   [menu.py] (OLED + Botones) ──► [main.py] (FSM Competencia)           │
 │                                        │                               │
 │                                        ▼                               │
 │                             [robot_hal.py] (HAL Multihilo)             │
 │                             - Hilo I2C: MPU-6050 + ToFs Láser          │
 │                             - Hilo UART: Transmisor hacia Arduino      │
 └───────────────────────────────────┬────────────────────────────────────┘
                                     │ Trama Serial UART (115200 Baud)
                                     │ "M,vel_izq,vel_der,servo\n"
                                     ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │                       ARDUINO UNO (src/arduino/)                       │
 │                                                                        │
 │   [firmware_motor_controller.ino]                                      │
 │   - Decodificación en tiempo real de consignas de velocidad            │
 │   - Generación determinista de PWM hacia driver DRV8833                │
 │   - Temporizador Watchdog: Si no hay trama en 300 ms -> Corte a cero  │
 └────────────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Guía de Ejecución y Despliegue

### 1. Flashear el Firmware en Arduino
1. Abre el Arduino IDE.
2. Carga el archivo `src/arduino/firmware_motor_controller/firmware_motor_controller.ino`.
3. Selecciona la placa **Arduino Uno** y el puerto COM asignado.
4. Presiona **Subir** (*Upload*).

### 2. Sincronizar el Software con la Orange Pi
Desde tu computadora de desarrollo conectada a la misma red local que la Orange Pi:
```cmd
cd src/orange
./sync.bat
```

### 3. Ejecutar Pruebas Modulares de Hardware
Conéctate por SSH a la Orange Pi y corre las pruebas individuales:
```bash
# Prueba de pantalla OLED
python3 ~/orange/tests/test_screen.py

# Prueba del sensor inercial
python3 ~/orange/tests/test_imu.py

# Prueba combinada IMU + pantalla
python3 ~/orange/tests/test_imu_screen.py
```

### 4. Lanzar la Simulación en Webots
En tu equipo con Webots instalado:
```cmd
cd src/webots-sim2real
launch.bat
```
