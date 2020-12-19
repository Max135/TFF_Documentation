#include <TimedAction.h>
#include <Adafruit_NeoPixel.h>

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_LSM6DS33.h> 
#include <Adafruit_SHT31.h> 
#include <Adafruit_Sensor.h> 

#define RGB_LED_PIN     8
#define LED_COUNT       1
#define PIN_OUT         10
#define PIN_IN          13
#define MIN             -15.5
#define MAX             15.5

#define MAX_TIME_LAST_HOOK 5000

void randomColor();
void setupRGB();
void setupVibrationSensor();
void vibrationDetected();
void getTemperature();
void getPressure();
void getCellphoneData();
void getHumidity();
void getHook();
void getVibration();
void getUserActions();

void sendData();
bool isAccelInTreshold(float, float, float);
void getHookDetected();

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

Adafruit_BMP280 bmp280; // temperature, barometric pressure
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30; // humidity


float temperature, pressure, altitude; 
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
String hookDetected;

short sampleBuffer[256]; // buffer to read samples into, each sample is 16-bits 
volatile int samplesRead; // number of samples read

Adafruit_NeoPixel strip(LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
boolean vibrationLastClock = false;
boolean hookDetectedLastClock = false;
unsigned long timeLastHook = millis();

TimedAction sendDataThread = TimedAction(5000, sendData);
TimedAction checkHookThread = TimedAction(100, getHookDetected);
TimedAction vibrationThread = TimedAction(1000, getUserActions);
//TimedAction tempThread = TimedAction(5000, getTemperature);
//TimedAction pressureThread = TimedAction(5000, getPressure);
//TimedAction humidityThread = TimedAction(5000, getHumidity);
TimedAction dataThread = TimedAction(10, getCellphoneData);

void setup() {
  Serial.begin(115200);
  setupSensors();
  setupRGB();
  setupVibrationSensor();

  setupBluefruit();
  startBledfu();
  setupBledis();
  startBleuart();
  setupBlebas();
  
  startAdv();

}

void loop() {
  sendDataThread.check();
  checkHookThread.check();
  vibrationThread.check();
//  randomColor();
}

void setupSensors() {
  bmp280.begin();
  lsm6ds33.begin_I2C();
  sht30.begin();
}

void setupBluefruit() {
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("TFF Smart Fishing");
  
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
}

void startBledfu() {
  bledfu.begin();
}

void setupBledis() {
  bledis.setManufacturer("Three Fisherman Friends");
  bledis.setModel("TFF SF-1.0");
  bledis.begin();
}

void startBleuart() {
  bleuart.begin();
}

void setupBlebas() {
  blebas.begin();
  blebas.write(100);
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void sendData() {
  getTemperature();
  getHumidity();
  getPressure();
}

void getUserActions() {
  getHook();
  getVibration();
  randomColor();
}

void getVibration() {
  digitalWrite(PIN_OUT, HIGH);
  if(vibrationLastClock) {
    Serial.println("Vibration detected");
    char buf[128];
    int ret = snprintf(buf, sizeof(buf), "%s", "true <vibration>");
    bleuart.write(buf, ret);
    vibrationLastClock = false;
  }
}

void getHook() {
  Serial.println(hookDetectedLastClock);
  if (hookDetectedLastClock) {
    char buf[128];
    int ret = snprintf(buf, sizeof(buf), "%s", "true <hook>");
    Serial.println(hookDetected);
    bleuart.write(buf, ret);
    hookDetectedLastClock = false;
  }
}

void getHookDetected() {
  if(millis() - timeLastHook > MAX_TIME_LAST_HOOK) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp; //temp needed by .getEvent for some reason
    lsm6ds33.getEvent(&accel, &gyro, &temp); 
    accel_x = accel.acceleration.x;
    accel_y = accel.acceleration.y; 
    accel_z = accel.acceleration.z; 
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;
    if (isAccelInTreshold(accel_x, accel_y, accel_z)) {
      Serial.println();
      Serial.print("Throw or hook : ");
      Serial.print("X ");
      Serial.print(accel_x);
      Serial.print(" ");
      Serial.print("Y ");
      Serial.print(accel_y); 
      Serial.print(" ");
      Serial.print("Z ");
      Serial.print(accel_z);
      hookDetectedLastClock = true;
      timeLastHook = millis();
    }
  }
}

bool isAccelInTreshold(float x, float y, float z) {
    return (x < MIN || x > MAX || y < MIN || y > MAX || z < MIN || z > MAX);
}

void getTemperature() {
  char buf[128];
  temperature = bmp280.readTemperature();
  int ret = snprintf(buf, sizeof(buf), "%f%s", temperature, "<temperature>");
  Serial.println();
  Serial.printf("Temperature: %f\n", temperature);
  bleuart.write(buf, ret);
  
}

void getHumidity() {
  char buf[128];
  humidity = sht30.readHumidity();
  int ret = snprintf(buf, sizeof(buf), "%f%s", humidity, "<humidity>");
  Serial.printf("Humidity: %f\n", humidity);
  bleuart.write(buf, ret);
}

void getPressure() {
  char buf[128];
  pressure = bmp280.readPressure() / 1000; // divided by 1000 to convert the pressure from pascals to kiloPascals (kPa)
  int ret = snprintf(buf, sizeof(buf), "%f%s", pressure, "<pressure>");
  Serial.printf("Pressure: %f\n", pressure);
  bleuart.write(buf, ret);
}

void getCellphoneData() {
    while ( bleuart.available() )
    {
      uint8_t ch;
      ch = (uint8_t) bleuart.read();
      Serial.write(ch);
    }
}

void connect_callback(uint16_t conn_handle) {
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

void setupVibrationSensor() {
  pinMode(PIN_OUT, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IN), vibrationDetected, RISING);
}

void vibrationDetected() {
  if(millis() - timeLastHook > MAX_TIME_LAST_HOOK) {
    vibrationLastClock = true;
  }
}

void setupRGB() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void randomColor() {
  strip.fill(strip.Color(random(20), random(20), random(20)), 0, 1);
  strip.show();
//  delay(500);
}
