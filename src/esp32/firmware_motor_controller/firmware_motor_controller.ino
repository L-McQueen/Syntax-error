/*
 * ==============================================================================
 * CONTROLADOR MAESTRO DE MOTORES Y SERVOMOTOR (ESP32-C3-ZERO / RISC-V 160 MHz)
 * ==============================================================================
 * Driver de Tracción: Toshiba TB6612FNG (Doble puente H MOSFET)
 * Comunicación: UART Serial a 115200 baud desde Orange Pi Zero 2W (Pin 11 -> GPIO 20)
 * Seguridad: Watchdog de parada de emergencia a 300 ms
 * PWM: LEDC a 20 kHz (inaudible, sin zumbido mecánico)
 * Calibración: Polaridad invertida en Motor B por montaje espejo en chasis
 * ==============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// --- ASIGNACIÓN DE PINES TB6612FNG (ESP32-C3-ZERO) ---
#define PIN_PWMA        0   // Motor Izquierdo Velocidad (PWM 20 kHz)
#define PIN_AIN1        1   // Motor Izquierdo Dirección 1
#define PIN_AIN2        2   // Motor Izquierdo Dirección 2
#define PIN_STBY        3   // Standby (HIGH = Habilitado)
#define PIN_BIN1        4   // Motor Derecho Dirección 1
#define PIN_BIN2        5   // Motor Derecho Dirección 2
#define PIN_PWMB        6   // Motor Derecho Velocidad (PWM 20 kHz)

#define PIN_SERVO       7   // Señal del micro-servo SG90 (Compuerta ArUco)
#define PIN_RGB_ZERO   10   // LED WS2812 integrado (Waveshare)
#define PIN_RGB_ALT     8   // LED integrado alternativo

// Pin RX por defecto en ESP32-C3: GPIO 20 (conectado a Pin 11 UART5_TX de Orange Pi)
// Pin TX por defecto en ESP32-C3: GPIO 21

// --- CONSTANTES DE OPERACIÓN ---
#define SERIAL_BAUD_RATE     115200
#define WATCHDOG_TIMEOUT_MS  300     // Límite sin datos antes de parada de emergencia (ms)
#define SERIAL_BUFFER_SIZE   64
#define PWM_FREQ_HZ          20000   // 20 kHz silencioso
#define PWM_RESOLUTION_BITS  8       // Rango de 0 a 255

// Calibración de Polaridad (Montaje en espejo en chasis diferencial)
#define INVERT_LEFT_MOTOR    false   // Motor Izquierdo normal
#define INVERT_RIGHT_MOTOR   true    // Motor Derecho invertido para emparejar avance

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
// FUNCIONES DE CONTROL DE MOTORES (TB6612FNG con PWM LEDC a 20 kHz)
// ==============================================================================

/**
 * Controla el Motor Izquierdo (Motor A)
 * speed: -255 a 255
 */
void setLeftMotor(int speed) {
  if (INVERT_LEFT_MOTOR) speed = -speed;
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
    // Parada suave (freewheel)
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
  }
}

/**
 * Controla el Motor Derecho (Motor B)
 * speed: -255 a 255
 */
void setRightMotor(int speed) {
  if (INVERT_RIGHT_MOTOR) speed = -speed;
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
    // Parada suave (freewheel)
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, 0);
  }
}

/**
 * Detiene inmediatamente ambos motores aplicando freno activo
 */
void stopMotors() {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  analogWrite(PIN_PWMA, 0);

  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  analogWrite(PIN_PWMB, 0);

  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

// ==============================================================================
// PROCESAMIENTO DEL PROTOCOLO SERIAL
// ==============================================================================

/**
 * Parsea el comando recibido desde Orange Pi: "M,<vel_izq>,<vel_der>,<servo_ang>"
 * Ejemplo: "M,150,150,90\n"
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
  // Configuración de pines de dirección TB6612FNG
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Despertar el chip TB6612FNG
  digitalWrite(PIN_STBY, HIGH);

  // Configurar frecuencia de PWM para silencio acústico a 20 kHz
  analogWriteFrequency(PIN_PWMA, PWM_FREQ_HZ);
  analogWriteResolution(PIN_PWMA, PWM_RESOLUTION_BITS);
  analogWriteFrequency(PIN_PWMB, PWM_FREQ_HZ);
  analogWriteResolution(PIN_PWMB, PWM_RESOLUTION_BITS);

  // Apagar motores al arranque
  stopMotors();

  // Inicializar micro-servo SG90 en GPIO 7
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

  Serial.println("BOOT_OK: ESP32_C3_ZERO_TB6612FNG_CONTROLLER_v2.0");
  lastCommandTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. VERIFICACIÓN DEL WATCHDOG DE SEGURIDAD (300 ms)
  // Si la Orange Pi deja de enviar comandos por más de 300ms, frenar motores inmediatamente
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
