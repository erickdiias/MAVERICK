#include <Wire.h>

// --- ENDEREÇO DO DISPOSITIVO ---
#define MPU6050_ADDR 0x68   // Endereço I2C do MPU6050

// --- REGISTRADORES DO MPU6050 ---
#define MPU6050_REG_ACCEL_XOUT_H 0x3B // Registrador inicial para leituras do acelerômetro
#define MPU6050_REG_PWR_MGMT_1   0x6B // Registrador de gerenciamento de energia
#define MPU6050_REG_GYRO_XOUT_H  0x43 // Registrador inicial para leituras do giroscópio

// --- ESCALAS DE SENSIBILIDADE ---
#define ACCEL_SCALE 16384.0 // Escala para acelerômetro em ±2g (16 bits)
#define GYRO_SCALE 131.0    // Escala para giroscópio em ±250°/s (16 bits)

// --- DESLOCAMENTOS DE CALIBRAÇÃO ---
double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0; // Offsets do acelerômetro
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;    // Offsets do giroscópio

// --- VARIÁVEIS DO FILTRO ---
double pitch = 0, roll = 0;   // Ângulos de orientação (em graus)
double dt = 0.02;             // Tempo de loop em segundos (50 Hz)

// --- PARÂMETROS DO FILTRO DE KALMAN ---
double Q_angle = 0.001;  // Ruído do processo para o acelerômetro
double Q_bias = 0.003;   // Ruído do processo para viés do giroscópio
double R_measure = 0.03; // Ruído da medição
double angle = 0, bias = 0, rate = 0; // Variáveis de estado do filtro de Kalman
double P[2][2] = {{0, 0}, {0, 0}};    // Matriz de covariância de erro

unsigned long lastTime; // Armazena tempo da última iteração

void setup() {
  Serial.begin(115200); // Inicia comunicação serial para debug
  Wire.begin();         // Inicia comunicação I2C

  MPU6050_init();       // Inicializa o sensor MPU6050
  calibrate_MPU6050();  // Calibra acelerômetro e giroscópio

  lastTime = millis();  // Define o tempo inicial do loop
}

void loop() {
  double ax, ay, az, gx, gy, gz;

  // --- LEITURA DO SENSOR ---
  read_MPU6050(ax, ay, az, gx, gy, gz);

  // --- CÁLCULO DO TEMPO ---
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  if (dt == 0) dt = 0.001; // Evita divisão por zero
  lastTime = currentTime;

  // --- APLICA FILTRO DE KALMAN PARA PITCH E ROLL ---
  pitch = Kalman_filter(pitch, gx, atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI);
  roll = Kalman_filter(roll, gy, atan2(ay, az) * 180.0 / PI);

  // --- EXIBE RESULTADOS NO MONITOR SERIAL ---
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print("°  ");
  Serial.print("Roll: "); Serial.print(roll); Serial.println("°");

  delay(20); // Mantém frequência de 50 Hz
}

// --- INICIALIZAÇÃO DO MPU6050 ---
void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1); // Seleciona o registrador de energia
  Wire.write(0);                      // Acorda o sensor
  Wire.endTransmission(true);
}

// --- CALIBRAÇÃO DO MPU6050 ---
void calibrate_MPU6050() {
  Serial.println("Calibrando MPU6050...");
  double ax, ay, az, gx, gy, gz;
  int numSamples = 100;

  for (int i = 0; i < numSamples; i++) {
    read_MPU6050(ax, ay, az, gx, gy, gz);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    gyroXOffset += gx;
    gyroYOffset += gy;
    gyroZOffset += gz;
    delay(10);
  }

  accelOffsetX /= numSamples;
  accelOffsetY /= numSamples;
  accelOffsetZ /= numSamples;
  gyroXOffset /= numSamples;
  gyroYOffset /= numSamples;
  gyroZOffset /= numSamples;

  // Ajusta a gravidade no eixo Z
  accelOffsetZ -= 1.0;

  Serial.println("Calibração do MPU6050 finalizada.");
}

// --- FUNÇÃO DE LEITURA DO MPU6050 ---
void read_MPU6050(double &ax, double &ay, double &az, double &gx, double &gy, double &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H); // Registrador inicial do acelerômetro
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  ax = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetX;
  ay = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetY;
  az = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetZ;
  gx = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroXOffset;
  gy = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroYOffset;
  gz = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroZOffset;
}

// --- FILTRO DE KALMAN ---
double Kalman_filter(double angle, double gyroRate, double accelAngle) {
  // Previsão
  rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Atualização
  double S = P[0][0] + R_measure; // Erro estimado
  double K[2];                    // Ganho de Kalman
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  double y = accelAngle - angle; // Diferença de ângulo
  angle += K[0] * y;
  bias += K[1] * y;

  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}
