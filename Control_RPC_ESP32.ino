#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ==========================================
// ESP32 - PID DISCRETO EXACTO (CORREGIDO RANGO)
// ==========================================

// --- PINES DE MOTORES ---
const int pinSliderA = 26; const int pinSliderB = 27;
const int pinPanA = 14;    const int pinPanB = 12;
const int pinTiltA = 32;   const int pinTiltB = 33;

// --- CONFIGURACIÓN PWM ---
const int PWM_FREQUENCY = 30000;
const int PWM_RESOLUTION = 8;

// --- VARIABLES DEL MODELO PID ---
double y = 0;       
double u = 0;       
double uP, uI, uD;  
double e = 0;       
double ekm1 = 0;    
double uIkm1 = 0;   

double r = 0;       

// --- GANANCIAS PID (RE-AJUSTADAS) ---
// Al duplicar el rango de grados, el error numérico crece, 
// así que ajustamos Kp ligeramente para mantener la misma fuerza.
double Kp = 1.6;    
double Ki = 1.4;   // Un poquito más de integral para asegurar llegada
double Kd = 0.6;    

// --- TIEMPO DE MUESTREO ---
double T = 0.04;    
unsigned long T_u;  
unsigned long t0;   

// --- HARDWARE ---
Adafruit_MPU6050 mpu;
enum MotorId { MOTOR_SLIDER=0, MOTOR_PAN=1, MOTOR_TILT=2 };

float u_slider = 0; 
float u_tilt = 0;

// --- CORRECCIÓN CLAVE AQUÍ ---
// Antes: 45.0 / 255.0 (Limitaba el giro a 45 grados)
// Ahora: 90.0 / 255.0 (Permite girar hasta 90 grados)
const float CMD_TO_DEG = 90.0 / 255.0; 

// Variables globales
float g_pan_deg = 0.0;
bool g_imu_ok = false;
uint32_t lastCmdTimeMs = 0;
const uint32_t CMD_TIMEOUT_MS = 2000;

// Prototipos
void setMotorFromU(MotorId id, double u_val);
void leerSensorIMU();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  // Configuración Pines
  ledcAttach(pinSliderA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinSliderB, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinPanA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinPanB, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinTiltA, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(pinTiltB, PWM_FREQUENCY, PWM_RESOLUTION);

  Wire.begin();
  if (!mpu.begin()) {
    g_imu_ok = false;
  } else {
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    g_imu_ok = true;
  }

  ekm1 = 0;
  uIkm1 = 0;
  
  T_u = T * 1000000;
  t0 = micros();
}

void loop() {
  // 1. LEER REFERENCIA
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      int c1 = data.indexOf(',');
      int c2 = data.indexOf(',', c1 + 1);
      
      if (c1 > 0 && c2 > c1 + 1) {
        int cmdSlider = data.substring(0, c1).toInt();
        int cmdPan    = data.substring(c1+1, c2).toInt();
        int cmdTilt   = data.substring(c2+1).toInt();

        // SLIDER y TILT (Directo - Aumenté un poco la ganancia para que acompañen)
        u_slider = cmdSlider * -4.0 / 255.0; 
        u_tilt   = cmdTilt   * -4.0 / 255.0;

        // PAN (PID Reference)
        // Ahora 'r' podrá llegar hasta 90 grados
        r = cmdPan * CMD_TO_DEG; 
        
        lastCmdTimeMs = millis();
      }
    }
  }

  // Timeout
  if (millis() - lastCmdTimeMs > CMD_TIMEOUT_MS) {
    u_slider = 0; u_tilt = 0; r = g_pan_deg; 
  }

  // 2. PID DISCRETO
  if (micros() - t0 >= T_u) {
    
    leerSensorIMU();
    y = g_pan_deg; 

    e = r - y;
    
    uP = Kp * e;
    uI = Ki * T * ekm1 + uIkm1; 
    uD = Kd * (e - ekm1) / T;
    
    u = uP + uI + uD;

    ekm1 = e;
    uIkm1 = uI;

    // Anti-windup ampliado ligeramente
    if (uIkm1 > 200) uIkm1 = 200;
    if (uIkm1 < -200) uIkm1 = -200;

    // 3. APLICAR A MOTORES
    setMotorFromU(MOTOR_PAN, u);    
    setMotorFromU(MOTOR_SLIDER, u_slider * 255.0); 
    setMotorFromU(MOTOR_TILT, u_tilt * 255.0);     

    // 4. FEEDBACK
    Serial.print("PID,");
    Serial.print(r); Serial.print(","); 
    Serial.print(y); Serial.print(","); 
    Serial.println(u);

    t0 = micros(); 
  }
}

void leerSensorIMU() {
  if (!g_imu_ok) return;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroZ = g.gyro.z; 
  if (abs(gyroZ) < 0.02) gyroZ = 0; 
  g_pan_deg += gyroZ * 57.296 * T; 
}

void setMotorFromU(MotorId id, double u_val) {
  int pinA, pinB;
  switch (id) {
    case MOTOR_SLIDER: pinA = pinSliderA; pinB = pinSliderB; break;
    case MOTOR_PAN:    pinA = pinPanA;    pinB = pinPanB;    break;
    case MOTOR_TILT:   pinA = pinTiltA;   pinB = pinTiltB;   break;
  }

  if (u_val > 255) u_val = 255;
  if (u_val < -255) u_val = -255;

  // Zona muerta
  if (abs(u_val) < 15) {
    ledcWrite(pinA, 0); ledcWrite(pinB, 0);
    return;
  }

  int duty = (int)abs(u_val);
  if (u_val > 0) {
    ledcWrite(pinA, duty); ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0); ledcWrite(pinB, duty);
  }
}