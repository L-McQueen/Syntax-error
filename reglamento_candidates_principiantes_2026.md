### Categoría Principiantes

Las reglas oficiales para la competencia "Candidates 2026" están sujetas a cambios, por lo que se les pide a los participantes estar atentos a cualquier modificación.

Cualquier duda favor de contactar a algún organizador.

*Versión actualizada del 28 de agosto de 2026*

## 1. Contenidos

1. Contenidos

2. Descripción del torneo

   2.1. Objetivo

   2.2. Participantes

   2.3. Competencia

   2.4. Términos a usar

3. Pistas

   3.1. Pista A: MAZE

   * 3.1.1. Objetivo

   * 3.1.2. Trayectoria de Pista

   * 3.1.3. Bonus Pista A

     3.2. Pista B: Niveles

   * 3.2.1. Objetivo

   * 3.2.2. Trayectoria de Pista

   * 3.2.3. Descripción sección 1

   * 3.2.4. Descripción sección 2

   * 3.2.5. Descripción sección 3

   * 3.2.6. Bonus Pista B

4. Puntuaciones

   4.1. Pista A

   4.2. Pista B

5. Lack of Progress

6. Especificaciones generales del robot

7. Reglas generales y Prohibiciones

8. Medidas y otras especificaciones

9. Seguimiento y bitácoras

   9.1. Sesiones de Seguimiento

   9.2. Bitácoras del Equipo

   9.3. Entregables

## 2. Descripción del torneo

### 2.1. Objetivo

Encontrar a alumnos destacados que se integren al equipo de robótica **RoBorregos** de acuerdo con su desempeño en la resolución del reto. Sin embargo, más allá de una selección, se busca compartir conocimientos de robótica dentro de la comunidad del Tecnológico de Monterrey y lograr un crecimiento tanto personal como profesional en los aplicantes.

### 2.2. Participantes

* El único requisito de admisión para la competencia Candidates Principiantes es que el estudiante esté cursando actualmente cualquier carrera del Tecnológico de Monterrey en su **1er, 2do o 3er semestre**.

* Para concursar en el torneo es necesario que los equipos se registren en tiempo y forma.

* Los equipos serán de **3 a 4 integrantes**. Trabajarán juntos en el diseño, construcción y programación de un robot que tenga las características especificadas en este reglamento.

* Es necesario que cada equipo designe a un **capitán**.

* Durante la competencia se espera que los participantes demuestren tanto habilidades técnicas como *soft skills* (trabajo en equipo, comunicación y liderazgo).

* Se espera que los participantes realicen todas las actividades de seguimiento indicadas por los organizadores del torneo.

### 2.3. Competencia

* La competencia está dividida en **tres rondas**, en las cuales cada equipo deberá participar de manera obligatoria.

* Por cada ronda los equipos tendrán que pasar por cada reto: al finalizar la ronda, los equipos habrán pasado por la **Pista A** y la **Pista B**.

* Los participantes tendrán por pista un tiempo total de **8 minutos**:

  * **Primeros 2 minutos:** Destinados a **calibración**. El equipo podrá hacer modificaciones de electrónica, mecánica o programación si así lo requieren, probar su robot en la pista, etc.

  * **6 minutos restantes:** Superación del reto de manera autónoma. Una vez iniciada la ronda, queda completamente prohibido cualquier tipo de modificación al robot.

#### Tiempo de calibración:

* Si el equipo lo desea, podrá solicitar al juez más tiempo de calibración consumiendo tiempo de ronda.

* La ronda comienza formalmente hasta que el juez o el capitán del equipo indique el inicio.

* Si el equipo finaliza la calibración antes de los 2 minutos, puede optar por iniciar la ronda; sin embargo, **el tiempo sobrante de calibración no es transferible a tiempo de ronda**.

* Si terminan los 2 minutos y el robot no está listo, pueden continuar calibrando a costa de consumir los 6 minutos de ronda.

* **Los puntos obtenidos durante el tiempo de calibración no cuentan para el puntaje de la ronda.**

### 2.4. Términos a usar

* **Calibración:** Ajuste de código de sensores, actuadores y posicionamiento del robot antes del inicio de la ronda. El equipo puede modificar electrónica o mecánica según considere necesario para el correcto funcionamiento del robot.

* **Lack of progress:** Situación en la que el robot es incapaz de avanzar / está atorado y se necesita reiniciar su posición (ver Sección 5).

* **Unidad:** Cuadrícula individual de $30 \times 30 \pm 2\text{ cm}$ que se toma como medida estándar para el diseño y secciones de la pista.

* **Checkpoint:** Punto al que el robot puede regresar si declara un *lack of progress*. Para que el checkpoint sea válido y se pueda utilizar como punto de reinicio, es necesario que por lo menos el **50% del robot se encuentre dentro de la casilla**.

* **Obstáculos:** Se consideran obstáculos: paredes, *speedbumps*, escaleras y/o rampas.

## 3. Pistas

* Las pistas asignadas a cada reto están separadas (no conectadas entre sí por checkpoints o rampas).

* Hay en total cuatro pistas: dos pistas A y dos pistas B. Las pistas A y B contienen retos diferentes que el robot debe resolver de manera autónoma.

* Una vez que el robot complete una pista, no será posible regresar al inicio o a cualquier checkpoint y se declara fin de ronda.

* **Paredes:** Las paredes exteriores de las pistas, las que rodean las rampas y las utilizadas en las Pistas A y B, tienen una **altura mínima de 15 cm**. Está estrictamente prohibido que los robots pasen por encima de las paredes.

* **Checkpoints:** Existe al menos un checkpoint por pista (incluyendo el final), los cuales otorgan puntos y sirven para relocalizar el robot en un *lack of progress*. Si no ha alcanzado ningún checkpoint, la casilla de inicio actúa como uno.

* Entre unidades con colores diferentes existe cierta separación de color blanco.

* Las dimensiones se mantienen fijas en todas las rondas, variando únicamente la distribución de elementos (checkpoints, obstáculos, *tiles* de colores y objetos a encontrar).

* Se hace uso paralelo de pistas entre diferentes equipos durante la ronda.

### 3.1. Pista A: MAZE

#### 3.1.1. Objetivo

Superar un laberinto de dimensiones de $5 \times 5$ **unidades**. El robot debe explorar por completo la pista y durante el proceso detectar colores ubicados en el suelo, superando obstáculos como *speedbumps*, escaleras y rampas.

#### 3.1.2. Trayectoria de Pista

* Inicia en el recuadro verde y concluye en el recuadro rojo (las posiciones pueden cambiar por ronda).

* El robot es libre de tomar la trayectoria más conveniente.

* **Restricción de obstáculos:** No habrá una escalera seguida de una rampa o una rampa seguida de una escalera.

* Habrá un máximo de:

  * 1 rampa

  * 3 escaleras

  * 4 speedbumps

* **Validación de detección de color:** Para considerar los puntos de detección, el robot debe indicarlo visualmente mediante un LED de ese color, display, pantalla, etc. **La emisión de sonido está estrictamente prohibida.**

* Se manejarán 4 colores a detectar en el suelo.

#### 3.1.3. Bonus Pista A

Los bonus son opcionales e independientes:

1. **Bonus 1 - Recorrido de regreso:** Otorga puntos extra solo si el robot llega al checkpoint final y regresa por el mismo camino recorrido hasta el checkpoint inicial.

   * Debe pasar por las mismas unidades en orden inverso.

   * No es necesario volver a detectar los colores.

   * Si se desvía de la trayectoria original, se cancela el bonus y termina la ronda.

2. **Bonus 2 - Detección de marcador ArUco (Visión Computacional):**

   * Se colocará un marcador ArUco (diccionario `4x4_50`, $\approx 8 \times 8\text{ cm}$) montado verticalmente en una pared.

   * El robot debe detectarlo con cámara durante el recorrido (sin necesidad de detenerse).

   * Debe reportar correctamente el ID del marcador al juez mediante un indicador visible (pantalla o secuencia de LEDs) durante **por lo menos 3 segundos continuos**.

   * *Prohibido indicar el ID mediante puerto serial o Bluetooth hacia una computadora.*

   * Si falla o reporta un ID incorrecto, no recibe puntos de bonus, pero no hay penalización sobre el resto del recorrido ni sobre el Bonus 1.

### 3.2. Pista B: Niveles

#### 3.2.1. Objetivo

Superar 3 secciones implementando distintas técnicas de programación, mecánica y electrónica.

#### 3.2.2. Trayectoria de Pista

Inicia en la casilla verde, cruza la Sección 1 hasta el primer checkpoint rojo, avanza por la Sección 2 hasta el segundo checkpoint rojo, y finalmente completa la Sección 3 hasta la casilla final.

#### 3.2.3. Descripción Sección 1

* Cuadrícula de $3 \times 3$ unidades con una pelota en el centro (pelota de golf estándar, color naranja, diámetro de $42 \pm 2\text{ mm}$).

* La casilla central tiene 3 paredes y un único lado abierto de acceso.

* El robot debe ingresar, tomar posesión de la pelota y transportarla hasta el siguiente checkpoint.

* **Posesión válida:** Para considerarse posesión, $>50\%$ **del volumen de la pelota debe estar dentro del perímetro del robot/garra**.

* Si el robot pierde la pelota, esta se queda donde cayó (se puede intentar recuperar). Bajo ninguna circunstancia la pelota se reacomoda en su posición inicial.

#### 3.2.4. Descripción Sección 2

* Cuadrícula de $2 \times 4$ unidades verdes.

* Contiene líneas blancas de $2\text{ cm}$ de ancho con espacios de paso libre de $30\text{ cm}$ en el eje horizontal.

* El robot debe atravesar la sección manteniendo la posesión de la pelota sin tocar las líneas blancas.

* Se considera infracción si $>50\%$ **de una llanta pisa o cruza la línea blanca**, lo cual amerita un *lack of progress* inmediato.

#### 3.2.5. Descripción Sección 3

* Cuadrícula de casillas de colores que dictan la dirección obligatoria de movimiento hacia el tile final.

* Las direcciones son absolutas respecto a la pista (tomando como referencia el frente hacia donde apunta el checkpoint de entrada), no respecto a la orientación relativa del robot:

  * **Azul celeste:** Derecha ($\rightarrow$)

  * **Amarillo:** Izquierda ($\leftarrow$)

  * **Naranja:** Arriba / Adelante ($\uparrow$)

  * **Rosa:** Abajo / Atrás ($\downarrow$)

* Salirse por completo de una casilla del camino establecido amerita *lack of progress*.

#### 3.2.6. Bonus Pista B

Llegar al último checkpoint al final de la pista manteniendo la posesión de la pelota en la garra en todo momento, sin haberla soltado.

## 4. Puntuaciones

### 4.1. Pista A

| Acciones | Puntos | 
 | ----- | ----- | 
| Color detectado y mostrado correctamente (LEDs, pantalla...). Máx. 4 colores por ronda. | 5 por color detectado | 
| Sobrepasar por completo un speedbump. Máx. 4 speedbumps. | 5 por speedbump | 
| Superar cada escalera (subir y bajar). Máx. 3 escaleras. | 15 por escalera | 
| Cruzar la rampa (subir y bajar). | 20 | 
| Llegar al checkpoint final. | 25 | 
| **BONUS 1:** Regresar por el mismo camino recorrido hasta el checkpoint inicial. | 35 | 
| **BONUS 2:** Detección y despliegue correcto de marcador ArUco. | 30 | 

### 4.2. Pista B

| Acciones | Puntos | 
 | ----- | ----- | 
| Tomar posesión de la pelota en Sección 1 y sacarla de la unidad central | 15 | 
| Llegar al checkpoint de la Sección 1 **SIN** posesión de la pelota | 40 | 
| Llegar al checkpoint de la Sección 1 **CON** posesión de la pelota | 70 | 
| Llegar al checkpoint de la Sección 2 **SIN** posesión de la pelota | 40 | 
| Llegar al checkpoint de la Sección 2 **CON** posesión de la pelota | 70 | 
| Llegar al checkpoint final (Sección 3) **SIN** posesión de la pelota | 50 | 
| **BONUS:** Llegar al checkpoint final (Sección 3) **CON** posesión de la pelota | 85 | 

> **Nota:** La puntuación máxima posible en la Pista B es de **240 puntos** (cumpliendo con la posesión en todas las etapas y el bonus final).

## 5. Lack of Progress

Un *lack of progress* ocurrirá si:

1. El capitán del equipo lo declara.

2. El robot o algún miembro del equipo daña la pista.

3. Algún miembro del equipo toca la pista o el robot sin autorización del juez (el juez puede determinar descalificación o fin de ronda según el caso).

### Procedimiento de reinicio:

* El capitán debe apagar o reiniciar el robot siempre con el mismo método predefinido y comunicado al juez antes de iniciar.

* **Queda prohibido alterar mecánica, electrónica o software durante el reinicio.**

* El capitán reubica el robot en el checkpoint válido alcanzado más reciente (o casilla de inicio).

* El cronómetro de ronda **sigue corriendo en todo momento**.

### Regla de 3 / 4 Lack of Progress (Pista B):

* Si se acumulan **3 lack of progress en la misma sección**, al **4to lack of progress** se da la opción de avanzar directamente al checkpoint de la siguiente sección. No obstante, **no se otorgarán puntos por alcanzar dicho nuevo checkpoint**.

* **Gestión de la pelota tras un Lack of Progress:**

  * **Sección 1:** La pelota se reinicia en su posición inicial (escondida en la casilla central).

  * **Sección 2 y 3:** La pelota se coloca justo enfrente del robot en el checkpoint de reinicio.

  * **Salto de sección al 4to Lack of Progress:** La pelota se coloca frente al robot en el checkpoint de la sección siguiente.

## 6. Especificaciones generales del robot

* **Dimensiones:** Limitadas al tamaño de **una Unidad (**$30 \times 30 \times 30\text{ cm}$**)** en los tres ejes.

* **Componentes:** Sin restricciones de sensores, actuadores, baterías, ruedas o microcontroladores.

* **Materiales y peso:** Libres (a criterio del equipo).

* **Daño:** Prohibido el uso de herramientas o partes que dañen la pista o alteren las superficies.

* **Originalidad:** Una similitud de $\ge 80\%$ en diseño o programación con otro robot causará la **descalificación de ambos equipos**.

* **Autonomía:** Completamente autónomo. Prohibida toda interacción externa una vez iniciada la ronda (salvo reposicionamiento por falta de progreso a cargo del capitán).

* **Construcción:** El robot debe ser fabricado por los miembros del equipo. Prohibido usar robots comerciales prediseñados o kits de robótica educativa cerrados (e.g., Lego Mindstorms, Lego Spike, etc.).

* **Comunicación:** Prohibida la comunicación externa participante-robot durante la ronda (Bluetooth, WiFi, telemetría manual). Sí se permite la comunicación inalámbrica interna entre placas o componentes del propio robot.

## 7. Reglas generales y Prohibiciones

* Equipos de **mínimo 3 y máximo 4 integrantes**.

* Todo miembro del equipo debe comprender la lógica y el código del robot. Prohibido usar código de terceros sin comprensión o descargado de internet para resolver la lógica de las pistas.

* **Pre-mapeo estrictamente prohibido:** No se permite almacenar la pista, paredes o colores en memoria previa a la ronda. La navegación debe ser en tiempo real.

* El capitán puede dar por terminada la ronda en cualquier momento y conservar los puntos acumulados.

* Las decisiones del juez son inapelables.

* **Desempate:** En caso de empate en puntuación, el criterio de desempate será el **menor tiempo empleado** en las rondas.

* En cancha solo pueden estar **máximo 2 personas del equipo** durante el tiempo de ronda.

* Solo el capitán tiene permitido tocar y manipular el robot.

* Todos los entregables deben ser de autoría propia; se permite el uso de documentación externa siempre que esté citada y justificada.

* **Política sobre Inteligencia Artificial:** En RoBorregos no se promueve el uso de IA generativa para programación, esquemáticos o diseño mecánico. El uso de IA no exime ni sustituye la capacidad de cada integrante de explicar, defender, justificar y modificar técnicamente sus soluciones.

## 8. Medidas y otras especificaciones

* Todas las medidas de la pista tienen una tolerancia de $\pm 2\text{ cm}$ (excepto las escaleras).

* Los tonos de color pueden variar ligeramente respecto a las ilustraciones.

* **Pelota de golf (Pista B):** Diámetro de $42 \pm 2\text{ mm}$, color naranja.

* **Cinta blanca (Pista B - Nivel 2):** Ancho de $2 \pm 1\text{ cm}$.

### Dimensiones de obstáculos (Pista A):

* **Speedbumps:**

  * Largo: $100\text{ mm}$

  * Ancho base: $20\text{ mm}$

  * Altura: $6\text{ mm}$

  * Ángulo lateral: $45^\circ$

* **Rampa:**

  * Subida/Inclinación: Longitud de $15 \pm 2\text{ cm}$ con pendiente de $15 \pm 2^\circ$

  * Meseta superior plana: $30 \pm 2\text{ cm}$

  * Bajada: Longitud de $15 \pm 2\text{ cm}$ con ángulo de $15 \pm 2^\circ$

* **Escalera:**

  * Ancho total de la unidad: $30 \pm 2\text{ cm}$

  * Ancho del peldaño superior: $20 \pm 2\text{ cm}$

  * Espacio de base lateral: $5 \pm 2\text{ cm}$

  * Altura por escalón: $0.5 \pm 0.5\text{ cm}$ (dos niveles de escalón)

## 9. Seguimiento y bitácoras

### 9.1. Sesiones de Seguimiento

Asistencia obligatoria semanal con mentores asignados para revisar avances.

### 9.2. Bitácoras del Equipo

Formato libre. Debe documentar detalladamente el progreso técnico y metodológico con evidencias (fotografías, videos, diagramas, fragmentos de código, enlaces a carpetas en Drive, GitHub, etc.).

### 9.3. Entregables

#### Programación

1. Repositorio de GitHub con el código fuente estructurado.

2. Bitácora técnica del desarrollo de software.

#### Electrónica

1. Diagrama esquemático en software EDA (EasyEDA, Altium, Proteus o KiCad).

2. Bill of Materials (BOM) con hipervínculos a los datasheets de los componentes.

3. Bitácora técnica del desarrollo de hardware.

4. *Opcionales:* Diseño y fabricación de PCB o ensamble en placa perforada.

#### Mecánica

1. Archivos CAD de piezas y ensamblajes (SolidWorks, Onshape o Fusion 360).

2. Renders o imágenes representativas desde distintos ángulos del robot y del ensamble CAD.

3. Bitácora técnica del diseño mecánico.

4. *Opcionales:* Planos técnicos normalizados y BOM de componentes mecánicos.

   ### 