/*
 * ==============================================================================
 * TEST COMPLETO: DUAL MOTORES TB6612FNG + SERVOMOTOR SG90 + ESP32-C3-ZERO
 * ==============================================================================
 * CABLEADO FÍSICO Y CÓDIGO DE COLORES:
 * 
 * ALIMENTACIÓN ESP32-C3-ZERO:
 * - 5V    ----> Entrada +5.00V regulada limpia desde Buck #2 (Cable NEGRO)
 * - GND   ----> Tierra común / GND (Cable BLANCO)
 * 
 * SERVOMOTOR SG90:
 * - SEÑAL ----> Pin GPIO 7 (p7) del ESP32-C3-Zero (Cable MORADO)
 * - VCC   ----> Riel +5.0V (Buck #2)
 * - GND   ----> Tierra común (GND)
 * 
 * CONTROL DRIVER TB6612FNG (Hacia ESP32-C3-Zero):
 * - PWMA  ----> GPIO 0 (Velocidad Motor Izquierdo - PWM 20 kHz)
 * - AIN1  ----> GPIO 1 (Dirección 1 Motor Izquierdo)
 * - AIN2  ----> GPIO 2 (Dirección 2 Motor Izquierdo)
 * - STBY  ----> GPIO 3 (Standby común: HIGH = Activo) - O puente a 3.3V
 * - BIN1  ----> GPIO 4 (Dirección 1 Motor Derecho)
 * - BIN2  ----> GPIO 5 (Dirección 2 Motor Derecho)
 * - PWMB  ----> GPIO 6 (Velocidad Motor Derecho - PWM 20 kHz)
 * - VCC   ----> 3.3V del ESP32 (Alimentación lógica del TB6612)
 * - GND   ----> Tierra lógica compartida (Cable Verde de Buck #2)
 * 
 * POTENCIA Y MOTORES (TB6612FNG):
 * - VM    ----> +7.48 V regulados desde Buck #1 (Cable Blanco)
 * - GND   ----> Tierra de potencia desde Buck #1 (Cable Negro)
 * - AO1 / AO2 ----> Cables del Motor Izquierdo (Motor A)
 * - BO1 / BO2 ----> Cables del Motor Derecho (Motor B)
 * ==============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// --- MAPEO DE PINES TB6612FNG ---
#define PIN_PWMA  0  // Motor Izquierdo PWM
#define PIN_AIN1  1  // Motor Izquierdo DIR 1
#define PIN_AIN2  2  // Motor Izquierdo DIR 2
#define PIN_STBY  3  // Standby compartido (HIGH = Habilitado)
#define PIN_BIN1  4  // Motor Derecho DIR 1
#define PIN_BIN2  5  // Motor Derecho DIR 2
#define PIN_PWMB  6  // Motor Derecho PWM

// --- MAPEO DEL SERVOMOTOR SG90 ---
#define PIN_SERVO 7  // Señal PWM del servomotor (Cable Morado)

// Pin LED RGB integrado WS2812
#define PIN_RGB_WAVESHARE 10
#define PIN_RGB_ALT        8

// Parámetros PWM silencioso
#define PWM_FREQ_HZ   20000  // 20 kHz (sin zumbido de bobina)
#define PWM_RES_BITS  8      // Rango 0 - 255

// Calibración de Polaridad (Inversión física por montaje espejo en chasis)
#define INVERT_MOTOR_A  false // Motor Izquierdo
#define INVERT_MOTOR_B  true  // Motor Derecho (Invertido para corregir sentido de giro)

// Objeto Servomotor
Servo gateServo;

// --- FUNCIONES DE CONTROL DE MOTORES ---

void setMotorA(int speed) {
  if (INVERT_MOTOR_A) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, -speed);
  } else {
    // Parada suave
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
  }
}

void setMotorB(int speed) {
  if (INVERT_MOTOR_B) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, -speed);
  } else {
    // Parada suave
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, 0);
  }
}

void frenarAmbos() {
  // Freno activo: cortocircuito dinámico de bobinas
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  analogWrite(PIN_PWMA, 0);

  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMB, 0);
}

void pararLibre() {
  // Reposo libre sin freno
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, 0);

  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMB, 0);
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  // Configuración de salidas digitales TB6612FNG
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Despertar el TB6612FNG
  digitalWrite(PIN_STBY, HIGH);

  // Frecuencia ultrasónica de 20 kHz en ambos canales PWM
  analogWriteFrequency(PIN_PWMA, PWM_FREQ_HZ);
  analogWriteResolution(PIN_PWMA, PWM_RES_BITS);
  analogWriteFrequency(PIN_PWMB, PWM_FREQ_HZ);
  analogWriteResolution(PIN_PWMB, PWM_RES_BITS);

  pararLibre();

  // Inicialización del Servomotor SG90 en GPIO 7 (p7)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);
  gateServo.write(90); // Posición inicial neutra / cerrado

  Serial.println("\n============================================================");
  Serial.println("  >>> TEST COMPLETO: MOTORES + SERVO (ESP32-C3-ZERO) <<<    ");
  Serial.println("============================================================");
  Serial.println("Alimentacion ESP32: 5V (Negro) | GND (Blanco)");
  Serial.println("Servomotor SG90:    GPIO 7 (p7 - Morado)");
  Serial.println("Motor Izq (A):      PWMA=GPIO 0 | AIN1=GPIO 1 | AIN2=GPIO 2");
  Serial.println("Motor Der (B):      PWMB=GPIO 6 | BIN1=GPIO 4 | BIN2=GPIO 5");
  Serial.println("Frecuencia PWM:     20 kHz silencioso");
  Serial.println("Iniciando secuencia de verificación en 2 segundos...\n");

  delay(2000);
}

void loop() {
  // 1. RECTO ADELANTE (Servo a 90° - Centro)
  Serial.println("[PASO 1] Ambos Motores ADELANTE | Servo: 90° (Centro)");
  neopixelWrite(PIN_RGB_WAVESHARE, 0, 50, 0); // Verde
  neopixelWrite(PIN_RGB_ALT, 0, 50, 0);
  gateServo.write(90);

  for (int v = 60; v <= 190; v += 10) {
    setMotorA(v);
    setMotorB(v);
    delay(40);
  }
  delay(1200);

  // 2. FRENO ACTIVO (Servo a 45°)
  Serial.println("[PASO 2] FRENO ACTIVO | Servo: 45°");
  neopixelWrite(PIN_RGB_WAVESHARE, 50, 0, 0); // Rojo
  neopixelWrite(PIN_RGB_ALT, 50, 0, 0);
  frenarAmbos();
  gateServo.write(45);
  delay(800);

  // 3. RECTO ATRAS (Servo a 135°)
  Serial.println("[PASO 3] Ambos Motores ATRAS (Reversa) | Servo: 135°");
  neopixelWrite(PIN_RGB_WAVESHARE, 0, 0, 50); // Azul
  neopixelWrite(PIN_RGB_ALT, 0, 0, 50);
  gateServo.write(135);

  for (int v = 60; v <= 190; v += 10) {
    setMotorA(-v);
    setMotorB(-v);
    delay(40);
  }
  delay(1200);

  // 4. FRENO ACTIVO
  frenarAmbos();
  delay(600);

  // 5. GIRO EN SU EJE A LA DERECHA (Servo a 180° - Total Abierto)
  Serial.println("[PASO 4] GIRO DERECHA sobre su eje | Servo: 180° (Abierto)");
  neopixelWrite(PIN_RGB_WAVESHARE, 40, 40, 0); // Amarillo
  neopixelWrite(PIN_RGB_ALT, 40, 40, 0);
  gateServo.write(180);
  setMotorA(160);
  setMotorB(-160);
  delay(1000);

  frenarAmbos();
  delay(600);

  // 6. GIRO EN SU EJE A LA IZQUIERDA (Servo a 0° - Total Cerrado)
  Serial.println("[PASO 5] GIRO IZQUIERDA sobre su eje | Servo: 0° (Cerrado)");
  neopixelWrite(PIN_RGB_WAVESHARE, 0, 40, 40); // Cian / Celeste
  neopixelWrite(PIN_RGB_ALT, 0, 40, 40);
  gateServo.write(0);
  setMotorA(-160);
  setMotorB(160);
  delay(1000);

  // 7. PARADA LIBRE Y REPOSO (Servo a 90° - Neutro)
  Serial.println("[PASO 6] CICLO COMPLETO - Reposo libre | Servo: 90°\n");
  neopixelWrite(PIN_RGB_WAVESHARE, 15, 15, 15); // Blanco tenue
  neopixelWrite(PIN_RGB_ALT, 15, 15, 15);
  gateServo.write(90);
  pararLibre();
  delay(2000);
}
