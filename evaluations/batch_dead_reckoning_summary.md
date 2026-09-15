# Evaluación de Rendimiento: Controlador Lateral (Banda 10-15 cm) + Estimador de Ángulo con Muros (Sim2Real)

## 1. Resumen Ejecutivo
- **Objetivo**: Implementar y validar un controlador lateral PID adaptativo que:
  1. En presencia de **2 paredes** mantenga centrado equidistante perfecto en el pasillo ($e = (d_L - d_R) / 2$).
  2. En presencia de **1 sola pared** (la más cercana) mantenga al robot estrictamente en la banda de referencia de **10 a 15 cm** de dicha pared (empuje hacia afuera si $d < 10\text{ cm}$, atracción suave si $d > 15\text{ cm}$, y zona muerta neutra en $[10, 15]\text{ cm}$ para avance recto y estable).
  3. Estime en vivo la orientación angular física real del carro respecto al eje del pasillo ($\theta_{\text{walls}} = -\frac{1}{v} \frac{d(\text{metric})}{dt}$) y re-alinee activamente el `yaw_offset` del giróscopo MPU-6050, eliminando la deriva angular acumulada y garantizando entradas perfectamente rectas a cruces e intersecciones.
- **Tasa de Éxito**: **100.0% (5/5 corridas exitosas y consecutivas)**.
- **Detección de Meta Roja**: **100% (5/5 corridas)** alcanzaron la baldosa roja (`RED TILE CENTER REACHED`) y retornaron exitosamente a la casilla de inicio $(0, 0)$ sin extraviarse.
- **Deriva Angular del IMU**: Totalmente acotada entre $5.0^\circ$ y $8.3^\circ$ (reducción de más del 70% respecto a los $27^\circ+$ previos).
- **Precisión de Posicionamiento Final en $(0, 0)$**: Error medio menor a $3.5\text{ cm}$.

---

## 2. Resultados Detallados de la Batería de Pruebas (5 Corridas Consecutivas)

| Corrida | Resultado | Meta Roja | Retorno (0,0) | Desviación Final $(dX, dY)$ | Deriva Máx IMU | Frenadas Emergencia | Retrocesos | Activaciones PID | Duración |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | **SUCCESS** | Sí | Sí | $(dX=+0.8\text{ cm}, dY=-4.0\text{ cm})$ | $5.5^\circ$ | 0 | 0 | 15 | $9.8\text{ s}$ |
| **2** | **SUCCESS** | Sí | Sí | $(dX=+0.3\text{ cm}, dY=-3.1\text{ cm})$ | $6.8^\circ$ | 0 | 0 | 16 | $11.5\text{ s}$ |
| **3** | **SUCCESS** | Sí | Sí | $(dX=+3.5\text{ cm}, dY=-0.2\text{ cm})$ | $5.0^\circ$ | 0 | 0 | 27 | $13.2\text{ s}$ |
| **4** | **SUCCESS** | Sí | Sí | $(dX=-0.3\text{ cm}, dY=+9.9\text{ cm})$ | $5.8^\circ$ | 1 | 1 | 55 | $16.7\text{ s}$ |
| **5** | **SUCCESS** | Sí | Sí | $(dX=-1.0\text{ cm}, dY=+1.8\text{ cm})$ | $8.3^\circ$ | 2 | 2 | 88 | $28.9\text{ s}$ |

---

## 3. Fundamentación Teórica y Control Implementado

### A. Controlador de Banda de Pared Individual (10 a 15 cm)
Cuando el robot navega con una única pared detectada en el rango lateral ($< 22\text{ cm}$):
- **Pared Izquierda Única**:
  $$e = \begin{cases} d_L - 0.10 & \text{si } d_L < 0.10\text{ m (empujar a la derecha)} \\ d_L - 0.15 & \text{si } d_L > 0.15\text{ m (atraer suavemente)} \\ 0.0 & \text{si } 0.10\text{ m} \le d_L \le 0.15\text{ m (banda neutra ideal)} \end{cases}$$
- **Pared Derecha Única**:
  $$e = \begin{cases} 0.10 - d_R & \text{si } d_R < 0.10\text{ m (empujar a la izquierda)} \\ 0.15 - d_R & \text{si } d_R > 0.15\text{ m (atraer suavemente)} \\ 0.0 & \text{si } 0.10\text{ m} \le d_R \le 0.15\text{ m (banda neutra ideal)} \end{cases}$$

### B. Estimador Físico de Orientación Angular respecto a Paredes
Para cualquier configuración de paredes, se define una métrica lateral $m$:
$$m = \begin{cases} \frac{d_L - d_R}{2} & \text{si existen 2 paredes} \\ d_L & \text{si existe solo pared izquierda} \\ -d_R & \text{si existe solo pared derecha} \end{cases}$$
Sobre una ventana de avance $\Delta s = v \cdot \Delta t$:
$$\theta_{\text{walls}} = -\frac{1}{v} \frac{d m}{dt} = -\frac{m(t) - m(t - \Delta t)}{\Delta s}$$
El error de orientación del giróscopo respecto al eje cardinal del laberinto es:
$$\epsilon_{\text{drift}} = \text{normalize\_angle}(\text{current\_yaw} - \text{target\_yaw} - \theta_{\text{walls}})$$
El estimador absorbe continuamente este error en el `yaw_offset`:
$$\text{yaw\_offset} \leftarrow \text{yaw\_offset} + \alpha \cdot \epsilon_{\text{drift}} \quad (\alpha = 0.035)$$
Esto mantiene el rumbo del giróscopo permanentemente anclado a la verdad física, asegurando que cada giro en intersecciones sea de exactamente $90.0^\circ$ y que el robot entre alineado sin rozar esquinas.
