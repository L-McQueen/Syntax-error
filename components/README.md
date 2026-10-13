# ⚙️ Guía Maestra de Componentes de Hardware

<div align="center">
  <p><strong>Análisis exhaustivo, hojas de datos, consumo eléctrico y justificación técnica de cada subsistema en 3nIgMa S3t3ki.</strong></p>
  <p>
    <img src="https://img.shields.io/badge/System-Distributed_SBC_%2B_MCU-blue.svg" alt="Architecture">
    <img src="https://img.shields.io/badge/Logic_Voltage-3.3V_%2F_5.0V-yellow.svg" alt="Logic">
    <img src="https://img.shields.io/badge/Motor_Voltage-7.8V_Regulated-red.svg" alt="Power">
    <img src="https://img.shields.io/badge/Buses-I2C_%7C_SPI_%7C_UART-brightgreen.svg" alt="Buses">
  </p>
</div>

---

> [!NOTE]
> Cada componente fue seleccionado rigurosamente bajo criterios de: inmunidad al ruido electromagnético, bajo consumo, determinismo en tiempo real y cumplimiento de las restricciones dimensionales y presupuestarias de **Candidates 2026**.

---

## 📑 Tabla Sinóptica de Especificaciones

| Componente | Rol en el Robot | Protocolo / Bus | Voltaje Lógico | Alimentación | Justificación Técnica Clave |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Orange Pi Zero 2W** | Computadora de Abordo (SBC) | Linux OS / Debian 12 | 3.3V | 5.0V / 2.0A | Procesamiento multinúcleo para visión, FSM y HAL multihilo |
| **Arduino Uno R3** | Controlador de Tracción (MCU) | UART (`/dev/ttyS5`) | 5.0V | 5.0V | Generación determinista de PWM sin latencia de SO |
| **DRV8833** | Driver Puente H Dual MOSFET | PWM (D3, D5, D6, D11) | 3.3V - 5.0V | 7.8V Estabilizado | Caída de voltaje casi nula ($\approx 0.1\text{V}$) frente al L298N |
| **Micromotores N20 (100:1)**| Actuadores de Tracción | Salidas Puente H | N/A | 7.8V (Potencia) | Alto torque con caja reductora metálica para subir rampas |
| **Pololu VL53L1X** | Sensor Frontal de Distancia | I2C (`/dev/i2c-2`, 0x29) | 3.3V | 3.3V / 5.0V | Rango láser largo de hasta 4.0 m con cono milimétrico estrecho |
| **Pololu VL53L0X (x2)** | Sensores Laterales ToF | I2C (0x30 y 0x31) | 3.3V | 3.3V / 5.0V | Medición precisa en pasillos de 30 cm sin eco acústico falso |
| **GY-521 (MPU-6050)** | Sensor Inercial (IMU 6-DOF) | I2C (`/dev/i2c-2`, 0x68) | 3.3V | 3.3V / 5.0V | Fusión trapezoidal en Yaw (Z) y filtro complementario de Pitch (Y) |
| **SSD1306 OLED 0.96"** | Interfaz Visual y Telemetría | I2C (`/dev/i2c-2`, 0x3C) | 3.3V | 3.3V / 5.0V | Diagnóstico en pista sin depender de WiFi ni monitores externos |
| **LM2596 Step-Down (x2)** | Regulación de Tensión DC-DC | Salidas fijas | N/A | Entrada 11.1V LiPo | Aislamiento galvánico: riel de motores (7.8V) vs riel lógico (5.0V) |
| **Cámaras (Webcam / Pixy)**| Visión Artificial Híbrida | USB (V4L2) / SPI | 5.0V / 3.3V | 5.0V | Detección ArUco por OpenCV y clasificación instantánea de color |

---

## 🔍 1. Orange Pi Zero 2W (Cerebro Principal)

* **Procesador (SoC):** Allwinner H618 Quad-Core ARM Cortex-A53 a 1.5 GHz.
* **Memoria:** 1 GB / 1.5 GB LPDDR4.
* **Conectividad:** Wi-Fi 5 (802.11ac) + Bluetooth 5.0 (usado únicamente para sincronización de código en pits, apagado en competencia).
* **Buses Integrados Usados:**
  - `/dev/i2c-2`: Conectado a IMU, pantalla OLED y los 3 sensores ToF.
  - `/dev/spidev1.0`: Bus SPI de alta velocidad para visión complementaria.
  - `/dev/ttyS5`: Puerto serial UART a 115200 baudios hacia el Arduino Uno.
* **Por qué se eligió:** Ofrece un costo 3 veces menor que una Raspberry Pi 4 con capacidad completa para ejecutar OpenCV, hilos concurrentes de sensado y algoritmos de grafo para laberintos.

---

## ⚡ 2. Arduino Uno R3 (Controlador de Potencia y Tiempo Real)

* **Microcontrolador:** Microchip ATmega328P a 16 MHz.
* **Función:** Generación de tren de pulsos PWM determinista y ejecución del Watchdog de seguridad (300 ms).
* **Por qué separar el control de motores:** Un sistema operativo como Linux no garantiza tiempo real estricto (*hard real-time*); si la SBC sufre un pico de CPU procesando visión, un PWM por software oscilaría, provocando desvíos. El microcontrolador garantiza estabilidad absoluta de frecuencia y ciclo útil.
* **Comunicación Simplex:** Recibe comandos en formato `M,vel_izq,vel_der,servo\n`. No transmite respuesta, eliminando necesidad de convertidores de nivel bidireccionales.

---

## 🚗 3. Driver DRV8833 vs. Puente H L298N

* **Tecnología:** Puentes H basados en MOSFETs integrados con protección térmica y por sobrecorriente.
* **Resistencia de Conducción:** $R_{DS(on)} \approx 0.2\ \Omega$.
* **Comparativa Crítica con L298N:**
  - El clásico L298N utiliza transistores bipolares BJT que tienen una caída interna de hasta $2.0\text{ V}$. En una batería de 7.8V, un L298N desperdicia casi el 25% de la energía disipando calor masivo.
  - El DRV8833 tiene una caída inferior a $0.1\text{ V}$, permitiendo transferir prácticamente el 100% del voltaje a los motores con tamaño minúsculo ($18 \times 15\text{ mm}$).

---

## 📏 4. Sensores de Distancia Láser ToF vs. Ultrasonidos HC-SR04

* **Sensor Frontal:** Pololu VL53L1X (Emisor láser VCSEL de 940 nm clase 1, alcance $4.0\text{ m}$).
* **Sensores Laterales:** Pololu VL53L0X (Alcance hasta $2.0\text{ m}$, optimizado para $5 - 30\text{ cm}$).
* **¿Por qué NO ultrasónicos HC-SR04?:**
  - El HC-SR04 emite un cono acústico de $15^\circ - 30^\circ$. En un laberinto con casillas de 30 cm, las ondas sonoras rebotan oblicuamente contra las paredes contiguas (*efecto multi-trayectoria*), generando lecturas fantasma.
  - El láser ToF mide la velocidad de la luz en un haz colimado de milímetros, garantizando mediciones ortogonales exactas independientemente del color o textura de la pared.

---

## 🧭 5. Sensor Inercial GY-521 (MPU-6050)

* **Ejes:** Giróscopo de 3 ejes ($\pm 250^\circ/\text{s}$ a $\pm 2000^\circ/\text{s}$) + Acelerómetro de 3 ejes ($\pm 2g$ a $\pm 16g$).
* **Frecuencia de Muestreo:** Bucle de lectura dedicado a 50 Hz.
* **Alineación Física en Chasis:**
  - Eje Y montado rígidamente en la dirección de avance longitudinal.
  - Eje Z vertical para integración de Yaw (guiñada) en giros.
* **Compensación de Sesgo (*Bias*):** Rutina de calibración en reposo durante los primeros 2 segundos de arranque con filtro de zona muerta a $0.25^\circ/\text{s}$.

---

## 📺 6. Pantalla OLED SSD1306 (0.96", 128x64 píxeles)

* **Interfaz:** I2C con dirección esclava `0x3C`.
* **Consumo:** $\approx 20\text{ mA}$ promedio (tecnología autoemisiva, no requiere luz de fondo).
* **Utilidad en Competencia:**
  - Selección de pista sin computadora (Pista A: Laberinto / Pista B: Desniveles).
  - Telemetría en vivo: ángulo Yaw, estado de rampa (SUBIDA / BAJADA / PLANO) y voltaje de batería.
  - Parada de emergencia y menú de diagnóstico de sensores en pits.

---

## 🔋 7. Distribución de Potencia: Batería LiPo 3S y Reguladores LM2596

* **Fuente Primaria:** Batería de Polímero de Litio (LiPo) 3S (11.1V nominal, 12.6V a plena carga).
* **Regulador LM2596 #1 (Potencia Motores):** Calibrado a $7.8\text{ V}$ continuos con condensador de filtro de bajo ESR. Alimenta exclusivamente el DRV8833.
* **Regulador LM2596 #2 (Lógica Limpia):** Calibrado a $5.0\text{ V}$ para la Orange Pi Zero 2W y el Arduino Uno R3.
* **Aislamiento Galvánico:** Esta separación física impide que los picos inductivos de los motores generen caídas momentáneas de voltaje (*voltage sags*), previniendo reinicios repentinos (*brownouts*) en el procesador.
