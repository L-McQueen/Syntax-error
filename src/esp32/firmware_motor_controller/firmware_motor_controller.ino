/*
 * Control de tracción DRV8833 y Micro-servo SG90 (ESP32-C3-Zero / RISC-V 160 MHz).
 * UART Serial a 115200 baud desde Orange Pi Zero 2W con Watchdog failsafe de 300 ms.
 * Lógica nativa de 3.3V compatible directa con Orange Pi Zero 2W.
 * PWM LEDC a 20 kHz (inaudible y sin zumbido de motor).
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// --- ASIGNACIÓN DE PINES ESP32-C3-ZERO ---
#define PIN_MOTOR_A_IN1 0   // Motor Izquierdo IN1 (LEDC PWM)
#define PIN_MOTOR_A_IN2 1   // Motor Izquierdo IN2 (LEDC PWM)
#define PIN_MOTOR_B_IN3 2   // Motor Derecho IN3 (LEDC PWM)
#define PIN_MOTOR_B_IN4 3   // Motor Derecho IN4 (LEDC PWM)

#define PIN_DRV_SLEEP   4   // DRV8833 Standby/Sleep (HIGH = Habilitado)
#define PIN_SERVO       5   // Señal del micro-servo SG90
#define PIN_LED_STATUS  8   // LED de estado en placa (GPIO 8)

// Pin RX por defecto en ESP32-C3: GPIO 20 (conectado a Pin 11 UART5_TX de Orange Pi)
// Pin TX por defecto en ESP32-C3: GPIO 21

// --- CONSTANTES DE OPERACIÓN ---
#define SERIAL_BAUD_RATE     115200
#define WATCHDOG_TIMEOUT_MS  300     // Límite sin datos antes de parada de emergencia (ms)
#define SERIAL_BUFFER_SIZE   64
#define PWM_FREQ_HZ          20000   // 20 kHz: alta frecuencia silenciosa
#define PWM_RESOLUTION_BITS  8       // Rango de 0 a 255

// --- OBJETOS Y VARIABLES GLOBALES ---
Servo gateServo;

unsigned long lastCommandTime = 0;
bool watchdogTripped = false;
char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t bufferIndex = 0;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int currentServoAngle = 90;

// ==============================================================================
// FUNCIONES DE CONTROL DE MOTORES (DRV8833 con PWM LEDC)
// ==============================================================================

/**
 * Controla el Motor Izquierdo (Motor A)
 * speed: -255 a 255
 */
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    analogWrite(PIN_MOTOR_A_IN1, speed);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    analogWrite(PIN_MOTOR_A_IN2, -speed);
  } else {
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
  }
}

/**
 * Controla el Motor Derecho (Motor B)
 * speed: -255 a 255
 */
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    analogWrite(PIN_MOTOR_B_IN3, speed);
    digitalWrite(PIN_MOTOR_B_IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_MOTOR_B_IN3, LOW);
    analogWrite(PIN_MOTOR_B_IN4, -speed);
  } else {
    digitalWrite(PIN_MOTOR_B_IN3, LOW);
    digitalWrite(PIN_MOTOR_B_IN4, LOW);
  }
}

/**
 * Detiene inmediatamente ambos motores a cero
 */
void stopMotors() {
  digitalWrite(PIN_MOTOR_A_IN1, LOW);
  digitalWrite(PIN_MOTOR_A_IN2, LOW);
  digitalWrite(PIN_MOTOR_B_IN3, LOW);
  digitalWrite(PIN_MOTOR_B_IN4, LOW);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

// ==============================================================================
// PROCESAMIENTO DEL PROTOCOLO SERIAL
// ==============================================================================

/**
 * Parsea el comando recibido: "M,<vel_izq>,<vel_der>,<servo_ang>"
 */
void parseCommand(char* cmd) {
  if (cmd[0] != 'M' || cmd[1] != ',') {
    return;
  }

  char* token;
  char* rest = cmd + 2; // Saltar "M,"

  token = strtok_r(rest, ",", &rest);
  if (!token) return;
  int speedLeft = atoi(token);

  token = strtok_r(rest, ",", &rest);
  if (!token) return;
  int speedRight = atoi(token);

  token = strtok_r(rest, ",", &rest);
  if (!token) return;
  int servoAngle = atoi(token);

  // Aplicar comandos a los actuadores
  setLeftMotor(speedLeft);
  setRightMotor(speedRight);
  
  servoAngle = constrain(servoAngle, 0, 180);
  gateServo.write(servoAngle);

  // Actualizar estado del Watchdog
  currentLeftSpeed = speedLeft;
  currentRightSpeed = speedRight;
  currentServoAngle = servoAngle;
  lastCommandTime = millis();
  watchdogTripped = false;

  // Enviar confirmación eco
  Serial.print("OK\n");
}

// ==============================================================================
// SETUP Y BUCLE PRINCIPAL (LOOP)
// ==============================================================================

void setup() {
  // Configuración de pines de motores
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN3, OUTPUT);
  pinMode(PIN_MOTOR_B_IN4, OUTPUT);

  // Configurar frecuencia de PWM para silencio acústico de motores
  analogWriteFrequency(PWM_FREQ_HZ);
  analogWriteResolution(PWM_RESOLUTION_BITS);

  // Pin Sleep del DRV8833
  pinMode(PIN_DRV_SLEEP, OUTPUT);
  digitalWrite(PIN_DRV_SLEEP, HIGH); // Habilitar puente H

  // LED de estado
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Apagar motores al arranque
  stopMotors();

  // Inicializar micro-servo SG90
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gateServo.setPeriodHertz(50); // Servo estándar a 50 Hz
  gateServo.attach(PIN_SERVO, 500, 2400); // 500us a 2400us estándar
  gateServo.write(currentServoAngle);

  // Puerto serial UART (GPIO 20 RX, GPIO 21 TX)
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 1000) {
    ; // Espera de inicialización
  }

  Serial.println("BOOT_OK: ESP32_C3_ZERO_DRV8833_CONTROLLER_v1.0");
  lastCommandTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. VERIFICACIÓN DEL WATCHDOG DE SEGURIDAD (300 ms)
  if (!watchdogTripped && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    stopMotors();
    watchdogTripped = true;
  }

  // 2. LECTURA ASÍNCRONA DEL BUFFER SERIAL
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        parseCommand(serialBuffer);
        bufferIndex = 0;
      }
    } else {
      if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[bufferIndex++] = c;
      } else {
        bufferIndex = 0; // Descarte ante desbordamiento
      }
    }
  }
}
