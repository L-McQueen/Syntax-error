# 📷 Ficha Técnica y Guía de Integración: Pixy2 (PixyCam v2)

Este documento condensa la documentación oficial de PixyCam ([Wiki Oficial](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start)), el mapeo físico con la **Orange Pi Zero 2W**, el protocolo de comunicación serial binario y la implementación en Python.

---

## 📌 1. Especificaciones Generales

| Parámetro | Especificación |
| :--- | :--- |
| **Procesador Onboard** | NXP LPC4330 dual-core (204 MHz Cortex-M4 + Cortex-M0) |
| **Sensor de Imagen** | Aptina MT9M114 (1296 × 976) con procesador de flujo de imagen |
| **Velocidad de Cuadros** | **60 FPS** (procesamiento de visión en tiempo real dentro de la cámara) |
| **Consumo de Corriente** | ~140 mA típico (hasta 400 mA con la lámpara LED integrada al 100%) |
| **Alimentación (VCC)** | 4.5 V a 6.0 V (Recomendado: 5.0 V limpio desde Buck #2) |
| **Niveles Lógicos I/O** | Salidas a **3.3 V CMOS** | Entradas tolerantes a 5 V |
| **Interfaces Soportadas** | **SPI** (hasta 2 Mbps), **USB**, **I2C**, **UART** (hasta 230 kbps), Analógico/Digital |
| **Iluminación Integrada** | Lámpara LED blanca de alto brillo para iluminar pasillos oscuros |
| **LED RGB de Estado** | Indicador configurable de color del objeto detectado |

---

## 🔌 2. Conector I/O de 10 Pines (IDC) y Conexión con Orange Pi Zero 2W

La Pixy2 cuenta con un conector hembra IDC de 10 pines (2x5 a 2.54 mm).

### Diagrama del Conector de Pixy2 (Vista frontal del conector):
```text
      +---+---+
SPI MISO | 1 | 2 | 5V (Entrada Alimentación)
 SPI SCK | 3 | 4 | SPI MOSI / UART TX
 I2C SCL | 5 | 6 | GND (Tierra)
  SPI SS | 7 | 8 | Analógico Out / DAC
 I2C SDA | 9 | 10| Sin Conexión
      +---+---+
```

### Tabla de Cableado SPI con la Orange Pi Zero 2W:
> Conexión recomendada para el bus `/dev/spidev1.0` de la Orange Pi (Alta velocidad a 2 MHz):

| Pin Pixy2 | Señal Pixy2 | Destino en Orange Pi Zero 2W | Pin Físico Orange Pi |
| :---: | :--- | :--- | :---: |
| **Pin 1** | **SPI MISO** | `PH8` (SPI1_MISO) | **Pin 21** |
| **Pin 2** | **5V Power** | Riel regulado 5.0V limpio (Buck #2) | Pin 2 o 4 (o salida Buck) |
| **Pin 3** | **SPI SCK** | `PH6` (SPI1_CLK) | **Pin 23** |
| **Pin 4** | **SPI MOSI** | `PH7` (SPI1_MOSI) | **Pin 19** |
| **Pin 6** | **GND** | Tierra común (GND) | **Pin 25** (o cualquier GND) |
| **Pin 7** | **SPI SS (CS)** | `PH5` (SPI1_CS0) | **Pin 24** |

*(Nota: En PixyMon, asegurarse de que en la pestaña "Interface" el parámetro "Data out port" esté configurado como **SPI with SS** si se usa el Chip Select del Pin 24).*

---

## 📡 3. Protocolo de Comunicación Serial Binario

Pixy2 utiliza un protocolo binario estructurado tipo petición-respuesta (*Request / Response*):

### Estructura de Paquetes:
1. **Paquete de Solicitud (Host -> Pixy2):**
   * `0xAE 0xC1`: 2 bytes de sincronización sin checksum (`0xC1AE` en Little-Endian).
   * `Tipo` (1 byte): Código del comando (ej. `0x0E` para versión, `0x20` para bloques de color).
   * `Longitud` (1 byte): Número de bytes de datos enviados.
   * `Datos`: Carga útil de parámetros.

2. **Paquete de Respuesta (Pixy2 -> Host):**
   * `0xAF 0xC1`: 2 bytes de sincronización con checksum (`0xC1AF`).
   * `Tipo` (1 byte): Código de respuesta.
   * `Longitud` (1 byte): Tamaño de los datos recibidos.
   * `Checksum` (2 bytes): Suma de comprobación de los datos.
   * `Datos`: Información de visión procesada.

---

## 🎯 4. Modos Operativos Principales

### A. Detección de Bloques de Color (Color Connected Components - CCC)
* Rastrea hasta **7 firmas de color individuales** (Firma 1 a 7) y **Códigos de Color** (combinaciones de 2 o más colores juntos para identificar puertas o balizas).
* Para cada objeto detectado entrega:
  * `Signature`: ID de color (1 a 7).
  * `X, Y`: Coordenadas del centro del objeto (Rango: X = 0 a 315, Y = 0 a 207).
  * `Width, Height`: Ancho y alto del bloque en píxeles.
  * `Angle`: Orientación del objeto si es un código de color.
  * `Tracking Index`: ID persistente para seguir un objeto cuadro a cuadro.

### B. Seguimiento de Líneas y Cruces (Line Tracking)
* Detecta líneas negras o blancas sobre el piso a 60 FPS.
* Identifica bifurcaciones en Y, intersecciones en T y cruces en cruz (+).
* Lee **códigos de barras en el suelo** (valores del 0 al 15) para instrucciones de giro o zonas de penalización.

### C. Control de Lámpara LED y Ajustes
* **Lámpara Blanca Superior:** Se puede encender mediante comando serial (`setLamp(upper, lower)`) para iluminar zonas oscuras del laberinto sin requerir luz ambiental externa.
* **Brillo de Cámara:** Ajuste dinámico de exposición para evitar deslumbramientos.

---

## 🐍 5. Ejemplo Básico en Python (Orange Pi Zero 2W con `spidev`)

```python
import spidev
import time

class Pixy2SPI:
    def __init__(self, bus=1, device=0, speed_hz=2000000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 3  # Pixy2 requiere SPI Modo 3

    def get_version(self):
        # Trama de solicitud de versión: [0xAE, 0xC1, 0x0E, 0x00]
        request = [0xAE, 0xC1, 0x0E, 0x00]
        self.spi.xfer2(request)
        time.sleep(0.001)
        # Leer cabecera de respuesta (6 bytes) + datos
        response = self.spi.xfer2([0x00] * 22)
        if response[0] == 0xAF and response[1] == 0xC1:
            print("[OK] Pixy2 Conectada y Respondiendo.")
            return response
        return None

    def set_lamp(self, upper_on=True, lower_on=False):
        # Comando 0x16: setLamp
        upper = 1 if upper_on else 0
        lower = 1 if lower_on else 0
        cmd = [0xAE, 0xC1, 0x16, 0x02, upper, lower]
        self.spi.xfer2(cmd)

    def close(self):
        self.spi.close()

if __name__ == "__main__":
    cam = Pixy2SPI()
    cam.get_version()
    cam.set_lamp(upper_on=True)
    time.sleep(1)
    cam.set_lamp(upper_on=False)
    cam.close()
```

---

## 🔗 6. Enlaces Oficiales y Descargas
* **Documentación y Wiki:** [docs.pixycam.com/wiki/doku.php?id=wiki:v2:start](https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:start)
* **Descarga de PixyMon v2 (Software de calibración para Windows/Linux):** [pixycam.com/downloads-pixy2](https://pixycam.com/downloads-pixy2/)
* **Repositorio Oficial de Código Fuente (libpixyusb2 y firmware):** [github.com/charmedlabs/pixy2](https://github.com/charmedlabs/pixy2)
