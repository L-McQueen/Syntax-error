/*
 * Control de tracción DRV8833 y Micro-servo SG90 (Arduino Uno R3 / ATmega328P).
 * UART Serial a 115200 baud desde Orange Pi Zero 2W con Watchdog failsafe de 300 ms.
 * Pines PWM: Motor A (D5, D6 - Timer 0), Motor B (D3, D11 - Timer 2), Servo (D9 - Timer 1).
 */

#include <Arduino.h>
#include <Servo.h>

// --- ASIGNACIÓN DE PINES ---
#define PIN_MOTOR_A_IN1 5   // Motor Izquierdo IN1 (Timer 0 PWM)
#define PIN_MOTOR_A_IN2 6   // Motor Izquierdo IN2 (Timer 0 PWM)
#define PIN_MOTOR_B_IN3 3   // Motor Derecho IN3 (Timer 2 PWM)
#define PIN_MOTOR_B_IN4 11  // Motor Derecho IN4 (Timer 2 PWM)

#define PIN_DRV_SLEEP   4   // DRV8833 Standby/Sleep (HIGH = Activo)
#define PIN_SERVO       9   // Señal del micro-servo SG90 (Timer 1)
#define PIN_LED_STATUS 13   // LED indicador integrado

// --- CONSTANTES DE OPERACIÓN ---
#define SERIAL_BAUD_RATE     115200
#define WATCHDOG_TIMEOUT_MS  300     // Tiempo límite sin datos antes de frenar (ms)
#define SERIAL_BUFFER_SIZE   64

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
// FUNCIONES DE CONTROL DE MOTORES (DRV8833)
// ==============================================================================

/**
 * Controla el Motor Izquierdo (Motor A)
 * speed: -255 a 255
 */
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    // Marcha hacia adelante
    analogWrite(PIN_MOTOR_A_IN1, speed);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
  } else if (speed < 0) {
    // Marcha hacia atrás
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    analogWrite(PIN_MOTOR_A_IN2, -speed);
  } else {
    // Frenado suave / Inercia (Coast)
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
    // Marcha hacia adelante
    analogWrite(PIN_MOTOR_B_IN3, speed);
    digitalWrite(PIN_MOTOR_B_IN4, LOW);
  } else if (speed < 0) {
    // Marcha hacia atrás
    digitalWrite(PIN_MOTOR_B_IN3, LOW);
    analogWrite(PIN_MOTOR_B_IN4, -speed);
  } else {
    // Frenado suave / Inercia (Coast)
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
    // Paquete inválido o no reconocido
    return;
  }

  // Parseo rápido de enteros usando strtok_r
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

  // Actualizar variables de estado
  currentLeftSpeed = speedLeft;
  currentRightSpeed = speedRight;
  currentServoAngle = servoAngle;
  lastCommandTime = millis();
  watchdogTripped = false;

  // Parpadeo rápido del LED indicador
  digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));

  // Enviar confirmación corta a la Orange Pi
  Serial.print(F("OK\n"));
}

// ==============================================================================
// SETUP Y BUCLE PRINCIPAL (LOOP)
// ==============================================================================

void setup() {
  // Configuración de pines de motores como salida
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN3, OUTPUT);
  pinMode(PIN_MOTOR_B_IN4, OUTPUT);

  // Pin Sleep del DRV8833
  pinMode(PIN_DRV_SLEEP, OUTPUT);
  digitalWrite(PIN_DRV_SLEEP, HIGH); // Habilitar puente H

  // Pin de LED indicador
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Asegurar que los motores arranquen completamente apagados
  stopMotors();

  // Inicializar micro-servo SG90
  gateServo.attach(PIN_SERVO);
  gateServo.write(currentServoAngle); // Iniciar en 90 grados (posición neutra/cerrada)

  // Inicializar puerto serial de hardware
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial && millis() < 1000) {
    ; // Espera máxima de 1 segundo para arranque
  }

  // Enviar mensaje de bienvenida en serial
  Serial.println(F("BOOT_OK: DRV8833_SERVO_CONTROLLER_v1.0"));

  lastCommandTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. VERIFICACIÓN DEL WATCHDOG DE SEGURIDAD
  if (!watchdogTripped && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    stopMotors();
    watchdogTripped = true;
    digitalWrite(PIN_LED_STATUS, LOW);
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
        // Desbordamiento de buffer: descartar para evitar corrupción
        bufferIndex = 0;
      }
    }
  }
}
