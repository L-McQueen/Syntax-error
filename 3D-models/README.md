# 🛠️ Diseño Mecánico y Modelado 3D (CAD)

<div align="center">
  <p><strong>Estructura paramétrica, cinemática diferencial, distribución de masas y tolerancias de manufactura aditiva para 3nIgMa S3t3ki.</strong></p>
  <p>
    <img src="https://img.shields.io/badge/CAD_Format-STEP_AP214-blue.svg" alt="STEP">
    <img src="https://img.shields.io/badge/Kinematics-Differential_Drive-orange.svg" alt="Kinematics">
    <img src="https://img.shields.io/badge/Material-PLA%2B_%2F_PETG-darkgreen.svg" alt="Material">
    <img src="https://img.shields.io/badge/Infill-35%25_Gyroid-purple.svg" alt="Infill">
  </p>
</div>

---

> [!NOTE]
> El archivo maestro [ensamble.step](ensamble.step) contiene el ensamblaje tridimensional completo del robot: chasis principal, torres de sensores ToF, soportes de micromotores N20, bandeja de batería LiPo y carcasa de la electrónica. Es compatible con cualquier software CAD estándar (Autodesk Fusion 360, SolidWorks, FreeCAD, Onshape e Inventor).

> [!TIP]
> Para lograr máxima rigidez contra impactos en las paredes del laberinto sin incrementar el peso por encima del límite reglamentario de 1.0 kg, se recomienda imprimir con 4 perímetros de pared y patrón de relleno giroide (*Gyroid*).

---

## 📐 1. Arquitectura Cinemática y Mecánica

El robot utiliza una configuración de **tracción diferencial de dos ruedas motrices y un punto de apoyo pasivo (caster ball)**, optimizada para radios de giro cero sobre su propio eje central y pasillos estrechos de casillas de 30 cm x 30 cm.

```
                            VISTA SUPERIOR DEL CHASIS
                                    [FRENTE]
                                       ▲
                            ┌─────────────────────┐
                            │    [VL53L1X ToF]    │  <- Sensor Frontal Largo Alcance
                            │        (Cámara)     │
                            │      ┌───────┐      │
             [VL53L0X Izq] ◄┤      │ Caster│      ├► [VL53L0X Der] (Pared Der)
             (Pared Izq)    │      │ Ball  │      │
                            │      └───────┘      │
                            │  [Pila LiPo 3S 11.1]│  <- Masa baja central
                            │    [Orange Pi 2W]   │
                            │    [Arduino Uno]    │
                            │  [DRV8833 + LM2596] │
                            │ ┌─────┐     ┌─────┐ │
             Rueda Izquierda│ │ N20 │     │ N20 │ │ Rueda Derecha
             Motriz (R=20mm)│ └─────┘     └─────┘ │ Motriz (R=20mm)
                            └─────────────────────┘
                                   [TRASERA]
```

### Especificaciones Dimensionales
* **Dimensiones Totales:** 142 mm (largo) x 128 mm (ancho) x 115 mm (alto) (cumple holgadamente el límite de 250 x 250 x 250 mm reglamentario).
* **Diámetro de Ruedas:** 40.0 mm (Radio $R = 20.0\text{ mm}$).
* **Trocha / Ancho de Vía ($W$):** 112.0 mm entre centros de rueda.
* **Masa Total Estimada:** 480 g (incluyendo batería LiPo 3S de 1000 mAh).
* **Centro de Gravedad ($CoG$):** Ubicado a 18 mm por delante del eje motriz y a solo 22 mm de altura respecto al suelo, lo que evita desbalanceos durante las transiciones de rampa en la Pista B.

---

## 🧱 2. Características del Chasis y Soportes de Sensores

1. **Bandeja Central para Batería:**
   * Diseñada con tope mecánico para paquetes LiPo 3S (dimensiones máximas 75 x 35 x 25 mm).
   * La batería se inserta desde la parte posterior con traba elástica, permitiendo cambios rápidos en zona de pits sin desmontar la electrónica ni desajustar tornillos.
2. **Bahías para Sensores de Tiempo de Vuelo (ToF):**
   * **Sensor Frontal VL53L1X:** Montado en la proa a 35 mm del suelo, con ángulo de 0° de elevación, blindado lateralmente con conducto cilíndrico para limitar el cono de dispersión óptica a $27^\circ$.
   * **Sensores Laterales VL53L0X:** Montados a 90° exactos perpendicular al eje longitudinal, orientados hacia las paredes laterales a la altura media de 35 mm (por encima del perfil de la base de madera de la pista).
3. **Bahía Aislada de Electrónica:**
   * Soporte en dos niveles: la Orange Pi Zero 2W se monta en el piso superior para facilitar la ventilación por convección del SoC Allwinner H618, mientras el Arduino Uno y el driver DRV8833 se fijan en el piso inferior cerca de las salidas de motor.
4. **Soporte Angular para OLED SSD1306:**
   * Inclinación a 45° en la cubierta superior para permitir la lectura cómoda de telemetría y el accionamiento de los pulsadores de navegación sin agacharse sobre la pista.

---

## 🖨️ 3. Parámetros de Manufactura Aditiva (Impresión 3D)

Todas las partes del archivo `ensamble.step` están optimizadas para manufactura FDM sin voladizos críticos mayores a 45°.

| Parámetro | Valor Recomendado | Justificación Técnica |
| :--- | :--- | :--- |
| **Material** | PLA+ o PETG | Resistencia mecánica, rigidez dimensional y nulo *warping*. |
| **Altura de Capa** | 0.20 mm | Equilibrio óptimo entre velocidad y precisión superficial. |
| **Perímetros de Pared** | 4 paredes (1.6 mm) | Resistencia torsional en caídas y choques frontales. |
| **Capas Superiores / Inferiores** | 5 superiores / 4 inferiores | Sellado robusto y soporte plano para PCBs. |
| **Densidad de Relleno** | 30% a 35% | Rigidez estructural sin añadir peso parásito. |
| **Patrón de Relleno** | Giroide (*Gyroid*) | Distribución isotrópica del esfuerzo en los 3 ejes cartesianos. |
| **Soportes** | Solo tocando placa (Orgánicos / Tree) | Fácil remoción sin dañar roscas ni conductos de sensores. |
| **Compensación de Agujeros** | +0.15 mm horizontal expansion | Ajuste holgado perfecto para tornillería M2 y M3. |

---

## 🔧 4. Tornillería y Ensamble Mecánico

* **Chasis a Electrónica:** Tornillos M3 x 6 mm de cabeza cilíndrica con separadores de nylon de 5 mm.
* **Sujeción de Micromotores N20:** Abrazaderas dobles impresas con tornillos M2 x 10 mm y tuercas M2 autoblocantes.
* **Placas ToF e IMU:** Tornillos autorroscantes M2 x 4 mm para fijación directa en torretas de plástico.
* **Ruedas:** Ruedas de goma de silicona de 40 mm con acople en D de 3 mm para eje N20.
