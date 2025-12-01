/*
S.P.A.R.K OS
Smart Pulse And Response Keeper 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <algorithm>

// DEFINICIONES Y CONFIGURACIÓN DE HARDWARE

// Registros del MAX30102
#define MAX30102_ADDR      0x57
#define REG_FIFO_DATA      0x07
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C
#define REG_LED2_PA        0x0D

// Asignación de pines del ESP32
#define BUZZER_PIN         25
#define LM35_PIN           34
#define SDA_PIN            21
#define SCL_PIN            22
#define AGE_BUTTON_PIN     2

// Configuración WiFi y Adafruit IO
#define WLAN_SSID          ""
#define WLAN_PASS          ""
#define AIO_SERVER         "io.adafruit.com"
#define AIO_SERVERPORT     1883
#define AIO_USERNAME       ""
#define AIO_KEY            ""

// Pantalla OLED
#define i2c_Address        0x3c
#define OLED_WIDTH         128
#define OLED_HEIGHT        64

// OBJETOS Y CLIENTES DE RED
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64, &Wire, -1);
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Canales MQTT para enviar datos a Adafruit IO
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/spark-temperatura");
Adafruit_MQTT_Publish hrFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/spark-ritmo");
Adafruit_MQTT_Publish spo2Feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/spark-spo2");

// VARIABLES GLOBALES - BUFFERS DE SENSORES
const int BUFFER_SIZE = 100;
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Variables de resultados finales
float lastHr = 0.0;
float lastSpo2 = 0.0;
float lastTemp = 0.0;
bool fingerPresent = false;

// VARIABLES GLOBALES - FLAGS
bool hasError = false;
bool errTempHigh = false;
bool errTempLow = false;
bool errHrHigh = false;
bool errHrLow = false;
bool errSpo2Low = false;

// VARIABLES GLOBALES - CONTROL DE TIEMPOS
unsigned long lastSensorReadTime = 0;
const int SENSOR_READ_INTERVAL = 50;  // Leer sensor cada 50ms

// VARIABLES GLOBALES - CONTROL DEL BOTÓN
unsigned long buttonDownTime = 0;
unsigned long lastButtonRelease = 0;
bool buttonActive = false;
bool longPressHandled = false;

// VARIABLES GLOBALES - MÁQUINA DE ESTADOS (Creditos al inge Enmanuel)
enum State { STATE_IDLE, STATE_WAIT_FINGER, STATE_READING, STATE_DONE, STATE_SHOW_ERRORS };
State currentState = STATE_IDLE;

// VARIABLES GLOBALES
int measurementCycle = 0;
float hrResults[3];
float spO2Results[3];
float tempResults[3];
int currentAgeRange = 0;
const char* ageNames[] = {"Baby", "Child", "Youth", "Adult"};
const int NUM_AGE_RANGES = 4;

struct VitalThresholds {
  int hrMin, hrMax;
  float tempMin, tempMax, spo2Min;
};

//Sistema para las edades
VitalThresholds thresholds[NUM_AGE_RANGES] = {
  { 80,  160, 36.5, 37.5, 94.0 },  // Bebé
  { 60,  100, 36.0, 37.0, 94.0 },  // Niño
  { 60,  100, 36.5, 37.2, 94.0 },  // Joven
  { 60,  110, 36.5, 37.2, 94.0 }   // Adulto Mayor
};

// COMUNICACIÓN I2C CON MAX30102 CON FUNCIONES

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readFifoData(uint32_t *red, uint32_t *ir) {
  Wire.beginTransmission(MAX30102_ADDR);
  Wire.write(REG_FIFO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30102_ADDR, 6);
  
  uint32_t un_red = ((Wire.read() & 0x03) << 16) | (Wire.read() << 8) | Wire.read();
  uint32_t un_ir = ((Wire.read() & 0x03) << 16) | (Wire.read() << 8) | Wire.read();
  
  *red = un_red;
  *ir = un_ir;
}

void setupMAX30102() {
  writeRegister(0x09, 0x40); 
  delay(100);
  writeRegister(0x02, 0xC0);
  writeRegister(0x09, 0x03);
  writeRegister(0x0A, 0x27);
  writeRegister(0x0C, 0x24);
  writeRegister(0x0D, 0x24);
}

void setSensorPower(bool on) {
  if (on) {
    writeRegister(0x09, 0x03);  // Modo SpO2 (Activo)
  } else {
    writeRegister(0x09, 0x80);  // Modo Shutdown (Bajo consumo)
  }
}

void beep(int duration_ms) {
  digitalWrite(BUZZER_PIN, LOW);   // Buzzer: Active LOW
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, HIGH);
}

// FUNCIONES DE RED: WiFi Y MQTT
void MQTT_connect() {
  if (mqtt.connected()) return;
  
  int8_t ret;
  if ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    mqtt.disconnect();
  }
}

void sendDataToCloud() {
  MQTT_connect();
  if (mqtt.connected()) {
    tempFeed.publish(lastTemp);
    hrFeed.publish(lastHr);
    spo2Feed.publish(lastSpo2);
    Serial.println(">> Datos enviados a Adafruit IO");
  }
}

// FUNCIONES DE CÁLCULO Y ALGORITMOS DE SIGNOS VITALES
int detectBPM(uint32_t *irData, int len) {
  float mean = 0;
  for (int i = 0; i < len; i++) {
    mean += irData[i];
  }
  mean /= len;
  
  int peaks = 0;
  for (int i = 1; i < len - 1; i++) {
    if (irData[i] > mean && irData[i] > irData[i - 1] && irData[i] > irData[i + 1]) {
      peaks++;
    }
  }
  
  return peaks * 12;
}

float calculateSpO2(uint32_t *irData, uint32_t *redData, int len) {
  float meanIR = 0, meanRed = 0;
  
  for (int i = 0; i < len; i++) {
    meanIR += irData[i];
    meanRed += redData[i];
  }
  meanIR /= len;
  meanRed /= len;
  
  float acIR = 0, acRed = 0;
  for (int i = 0; i < len; i++) {
    acIR += (irData[i] - meanIR) * (irData[i] - meanIR);
    acRed += (redData[i] - meanRed) * (redData[i] - meanRed);
  }
  
  float rmsIR = sqrt(acIR / len);
  float rmsRed = sqrt(acRed / len);
  float R = (rmsRed / meanRed) / (rmsIR / meanIR);
  float spo2 = 110.0 - 25.0 * R;
  
  // Limitar valores a rango válido
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 70) spo2 = 70;
  
  return spo2;
}

bool isFingerDetected(uint32_t *irData, int len) {
  if (len == 0) return false;
  
  float meanIR = 0;
  for (int i = 0; i < len; i++) {
    meanIR += irData[i];
  }
  meanIR /= len;
  
  return meanIR > 5000;  // Umbral de detección
}

// FUNCIONES DE ESTADÍSTICA Y ANÁLISIS

float getMedian(float a, float b, float c) {
  float arr[] = {a, b, c};
  
  // Bubble sort simple
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
  
  return arr[1];
}

float getMax(float a, float b, float c) {
  float maxVal = a;
  if (b > maxVal) maxVal = b;
  if (c > maxVal) maxVal = c;
  return maxVal;
}

/*
Calcula los resultados finales usando mediana de 3 ciclos e imprime en terminal para depuración
 */

void calculateFinalResults() {
  lastHr = getMedian(hrResults[0], hrResults[1], hrResults[2]);
  lastTemp = getMedian(tempResults[0], tempResults[1], tempResults[2]);
  lastSpo2 = getMax(spO2Results[0], spO2Results[1], spO2Results[2]);
  
  // Imprime resultados en terminal
  Serial.println("\n MEDICIÓN COMPLETADA");>
  Serial.print("Rango de Edad: "); Serial.println(ageNames[currentAgeRange]);
  Serial.print("Temp: "); Serial.print(lastTemp); Serial.println(" C");
  Serial.print("SpO2: "); Serial.print(lastSpo2); Serial.println(" %");
  Serial.print("HR:   "); Serial.print(lastHr);   Serial.println(" BPM");
}

/*
Analiza los signos vitales y activa banderas de alerta si algún valor está fuera de rango para la edad seleccionada
 */
void analyzeVitals() {
  errTempHigh = errTempLow = errSpo2Low = errHrHigh = errHrLow = hasError = false;
  VitalThresholds current = thresholds[currentAgeRange];
  
  // Verificar cada signo vital contra sus límites
  if (lastSpo2 < current.spo2Min && lastSpo2 > 30) {
    errSpo2Low = true;
    hasError = true;
  }
  if (lastTemp > current.tempMax) {
    errTempHigh = true;
    hasError = true;
  }
  if (lastTemp < current.tempMin) {
    errTempLow = true;
    hasError = true;
  }
  if (lastHr > current.hrMax) {
    errHrHigh = true;
    hasError = true;
  }
  if (lastHr < current.hrMin) {
    errHrLow = true;
    hasError = true;
  }
  
  if (hasError) {
    Serial.println(">> ¡ALERTA DETECTADA!");
  }
}

// FUNCIONES DE INTERFAZ DE USUARIO - PANTALLA OLED

/*
Dibuja pantalla principal mostrando estado actual del sistema Cambia contenido según el estado de la máquina de estados
*/
void drawMainScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  char buffer[20];
  
  // ESTADO: MIDIENDO
  if (currentState == STATE_READING) {
    display.setTextSize(2);
    display.setCursor(10, 10);
    display.print("MEASURING");
    
    // Barra de progreso
    display.drawRect(0, 48, 128, 12, SH110X_WHITE);
    int totalSamples = 300;
    int currentSamples = (measurementCycle * 100) + bufferIndex;
    int barWidth = map(currentSamples, 0, totalSamples, 0, 124);
    display.fillRect(2, 50, barWidth, 8, SH110X_WHITE);
    
    display.display();
    return;
  }
  
  // ESTADO: RESULTADOS
  if (currentState == STATE_DONE) {
    sprintf(buffer, "Temp: %.1f C", lastTemp);
    display.setCursor(0, 0);
    display.println(buffer);
    
    sprintf(buffer, "SpO2: %.1f %%", lastSpo2);
    display.setCursor(0, 12);
    display.println(buffer);
    
    sprintf(buffer, "HR: %.0f BPM", lastHr);
    display.setCursor(0, 24);
    display.println(buffer);
    
    // Botón para continuar
    display.drawRect(0, 48, 128, 16, SH110X_WHITE);
    display.setCursor(4, 52);
    display.print("CLICK TO CONTINUE");
    
    display.display();
    return;
  }
  
  // ESTADO: ESPERA
  display.setCursor(0, 0);
  display.println("Temp: ---");
  display.setCursor(0, 12);
  display.println("SpO2: ---");
  display.setCursor(0, 24);
  display.println("HR: ---");
  
  // Mostrar rango de edad actual
  display.setCursor(0, 36);
  sprintf(buffer, "Range: %s", ageNames[currentAgeRange]);
  display.print(buffer);
  
  // Instrucciones en barra inferior
  display.drawRect(0, 48, 128, 16, SH110X_WHITE);
  display.setCursor(4, 52);
  
  if (currentState == STATE_IDLE) {
    display.print("HOLD BTN TO START");
  } else if (currentState == STATE_WAIT_FINGER) {
    display.print("PLACE FINGER NOW");
  }
  
  display.display();
}

/*
Dibuja pantalla de alertas mostrando qué signos vitales están fuera de rango
*/
void drawErrorScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  
  // Título de alerta
  display.setCursor(30, 0);
  display.println("ALERT!");
  
  // Mostrar cada alerta activa
  display.setTextSize(1);
  int y = 20;
  
  if (errTempHigh) {
    display.setCursor(0, y);
    display.print("- High Temp: ");
    display.print(lastTemp, 1);
    y += 10;
  }
  if (errTempLow) {
    display.setCursor(0, y);
    display.print("- Low Temp: ");
    display.print(lastTemp, 1);
    y += 10;
  }
  if (errSpo2Low) {
    display.setCursor(0, y);
    display.print("- Low O2: ");
    display.print(lastSpo2, 1);
    y += 10;
  }
  if (errHrHigh) {
    display.setCursor(0, y);
    display.print("- High HR: ");
    display.print((int)lastHr);
    y += 10;
  }
  if (errHrLow) {
    display.setCursor(0, y);
    display.print("- Low HR: ");
    display.print((int)lastHr);
    y += 10;
  }
  
  // Botón para salir
  display.drawRect(0, 52, 128, 12, SH110X_WHITE);
  display.setCursor(20, 54);
  display.print("CLICK TO EXIT");
  
  display.display();
}

// CONFIGURACIÓN INICIAL

void setup() {
  // Configurar pines
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);  // Buzzer OFF (Active LOW)
  
  pinMode(AGE_BUTTON_PIN, INPUT_PULLUP);
  
  // Inicializar comunicación serial
  Serial.begin(115200);
  
  // Inicializar I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(250);
  
  // Inicializar pantalla OLED
  if (!display.begin(i2c_Address, true)) {
    Serial.println(F("OLED no detectada, continuando..."));
  }
  display.setRotation(0);
  
  // Pantalla de carga: conectando WiFi
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);
  display.println("Connecting WiFi...");
  display.display();
  
  // Conectar a WiFi
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  unsigned long startWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startWifi < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi OK");
  } else {
    Serial.println("WiFi Failed");
  }
  
  // Pantalla de inicio: Logo SPARK
  display.clearDisplay();
  display.setCursor(34, 15);
  display.setTextSize(2);
  display.println("SPARK");
  display.display();
  
  // Melodía de inicio
  beep(100);
  delay(50);
  beep(100);
  delay(50);
  beep(200);
  delay(1000);
  
  // Inicializar sensor MAX30102
  setupMAX30102();
  
  // Inicializar máquina de estados
  setSensorPower(false); 
  currentState = STATE_IDLE;
  drawMainScreen();
}

// BUCLE PRINCIPAL - MÁQUINA DE ESTADOS

void loop() {
  // Procesar mensajes MQTT si hay conexión
  if (millis() % 1000 == 0 && WiFi.status() == WL_CONNECTED) {
    mqtt.processPackets(10);
  }
  
  //LÓGICA DE LECTURA DEL BOTÓNN
  int buttonState = digitalRead(AGE_BUTTON_PIN);
  
  if (buttonState == LOW && !buttonActive) {
    // Botón recién presionado
    buttonActive = true;
    buttonDownTime = millis();
    longPressHandled = false;
  }
  
  // Detectar pulsación larga (1 segundo) para iniciar medición
  if (buttonState == LOW && buttonActive) {
    if ((millis() - buttonDownTime > 1000) && !longPressHandled) {
      if (currentState == STATE_IDLE) {
        beep(200);
        currentState = STATE_WAIT_FINGER;
        setSensorPower(true);
        drawMainScreen();
      }
      longPressHandled = true;
    }
  }
  
  // Detectar click (liberación del botón)
  if (buttonState == HIGH && buttonActive) {
    buttonActive = false;
    
    if (!longPressHandled && (millis() - lastButtonRelease > 400)) {
      lastButtonRelease = millis();
      
      // Click en IDLE: cambiar rango de edad
      if (currentState == STATE_IDLE) {
        currentAgeRange = (currentAgeRange + 1) % NUM_AGE_RANGES;
        beep(50);
        drawMainScreen();
      }
      // Click en DONE: revisar alertas
      else if (currentState == STATE_DONE) {
        beep(50);
        if (hasError) {
          currentState = STATE_SHOW_ERRORS;
          beep(500);  // Alerta sonora
          drawErrorScreen();
        } else {
          currentState = STATE_IDLE;
          setSensorPower(false);
          drawMainScreen();
        }
      }
      // Click en SHOW_ERRORS: salir de alertas
      else if (currentState == STATE_SHOW_ERRORS) {
        beep(50);
        currentState = STATE_IDLE;
        setSensorPower(false);
        drawMainScreen();
      }
    }
  }
  
  // ========== MÁQUINA DE ESTADOS ==========
  
  if (currentState == STATE_WAIT_FINGER) {
    // Esperar detección de dedo
    uint32_t ir, red;
    readFifoData(&red, &ir);
    
    if (ir > 5000) {
      // Dedo detectado
      currentState = STATE_READING;
      measurementCycle = 0;
      bufferIndex = 0;
      beep(100);
      drawMainScreen();
    }
  }
  
  else if (currentState == STATE_READING) {
    // Fase de medición (3 ciclos de 5 segundos)
    unsigned long currentTime = millis();
    
    if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
      lastSensorReadTime = currentTime;
      
      uint32_t ir, red;
      readFifoData(&red, &ir);
      irBuffer[bufferIndex] = ir;
      redBuffer[bufferIndex] = red;
      bufferIndex++;
      
      // Detectar si el dedo se retira durante la medición
      if (ir < 5000) {
        currentState = STATE_IDLE;
        setSensorPower(false);
        drawMainScreen();
        return;
      }
      
      // Actualizar barra de progreso
      if (bufferIndex % 10 == 0) {
        drawMainScreen();
      }
      
      // Procesar ciclo completo
      if (bufferIndex >= BUFFER_SIZE) {
        float cycleHr = detectBPM(irBuffer, BUFFER_SIZE);
        float cycleSpo2 = calculateSpO2(irBuffer, redBuffer, BUFFER_SIZE);
        float cycleTemp = (analogRead(LM35_PIN) / 4095.0) * 330.0;
        
        // Aplicar filtros de validación
        if (cycleHr < 40 || cycleHr > 220) cycleHr = 0;
        if (isnan(cycleTemp) || cycleTemp < 20 || cycleTemp > 45) cycleTemp = 0;
        if (cycleSpo2 == 0) cycleSpo2 = 0;
        
        hrResults[measurementCycle] = cycleHr;
        spO2Results[measurementCycle] = cycleSpo2;
        tempResults[measurementCycle] = cycleTemp;
        
        measurementCycle++;
        bufferIndex = 0;
        
        // Verificar si la medición está completa (3 ciclos)
        if (measurementCycle >= 3) {
          // Medición completada
          calculateFinalResults();
          analyzeVitals();  // Detectar alertas
          sendDataToCloud();  // Enviar datos a Adafruit IO
          
          currentState = STATE_DONE;
          beep(200);
          drawMainScreen();
        }
      }
    }
  }
}
