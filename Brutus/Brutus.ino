#include <AssetTracker.h>
#include <DS18B20.h>
#include <Adafruit_SSD1306.h>
#include <MQTT.h>
#include <blynk.h>

#define PARTICLE_KEEPALIVE 15

Adafruit_SSD1306 oled(-1);

const int      MAXRETRY          = 4;
const uint32_t msSAMPLE_INTERVAL = 2500;
const uint32_t msMETRIC_PUBLISH  = 30000;

DS18B20  ds18b20(D2, true);
char     szInfo[64];
double   celsius;
double   fahrenheit;
uint32_t msLastMetric;
uint32_t msLastSample;

double lat;
double lon;
bool hasGPSFix;
float battVoltage;
float battPercentage;
String messageGPSBlynk;

// Blynk Setup
#define BLYNK_IP IPAddress(188,166,206,43)
#define BLYNK_PRINT Serial  // Serial output for debug prints
#define BLYNK_DEBUG
// Set Blynk hertbeat interval.
// Each heartbeat uses ~90 bytes of data.
#define BLYNK_HEARTBEAT 60
// Blynk Auth Token
char auth[] = "059f483636b945a1a92eacdb33c3718c";
BlynkTimer timer;

#define BLYNK_VIRTUAL_PIN_SLIDE_TEMP_INPUT V1
#define BLYNK_VIRTUAL_PIN_LAT              V2
#define BLYNK_VIRTUAL_PIN_LON              V3
#define BLYNK_VIRTUAL_PIN_GPSMSG           V4
#define BLYNK_VIRTUAL_PIN_TEMPC            V5
#define BLYNK_VIRTUAL_PIN_TEMPH            V6
#define BLYNK_VIRTUAL_PIN_BATTVOLTAGE      V7
#define BLYNK_VIRTUAL_PIN_BATTPERCENTAGE   V8
#define BLYNK_VIRTUAL_PIN_MAP              V9
#define BLYNK_VIRTUAL_PIN_RELAY_LOCK       V10
#define BLYNK_VIRTUAL_PIN_ZEBRA            V12
#define BLYNK_VIRTUAL_PIN_TWITTER          V13

WidgetMap gpsMap(BLYNK_VIRTUAL_PIN_MAP);

int transmittingData = 1;
long lastPublish = 0;
int delayMinutes = 1;
AssetTracker t = AssetTracker();

int transmitMode(String command) {
  transmittingData = atoi(command);
  return 1;
}

int gpsPublish(String command) {
  if (t.gpsFix()) {
    Particle.publish("location", t.readLatLon(), 60, PRIVATE);
    return 1;
  } else {
    return 0;
  }
}

void callback(char* topic, byte* payload, unsigned int length);
byte PPServer[] = {128, 199, 157, 0 };

MQTT client(PPServer, 1883, callback);

void callback(char* topic, uint8_t* payload, unsigned int length) {

}

void setup() {
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();

  t.begin();
  t.gpsOn();

  Blynk.begin(auth, BLYNK_IP);
  if(Blynk.connected()) {
    Blynk.syncAll();
  }

  client.connect("mqtt_fin", "ppsmart", "ppmqtt");
  if (client.isConnected()) {
      client.publish("homeassistant/bfish","mqtt_fin Connected.");
    }

  Serial.begin(9600);

  Particle.function("tmode", transmitMode);
  Particle.function("gps", gpsPublish);
}

void loop() {
  if (client.isConnected())
  client.loop();
  Blynk.run();
  getTemp();
  getGPS();
  displayOled();
  oled.display();
}

void getGPS(){

   Serial.println(t.preNMEA());
  if(t.gpsFix()) {
    lat = t.readLat();
    lon = t.readLon();
    hasGPSFix = t.gpsFix();
    if(hasGPSFix) {
      messageGPSBlynk = "GPS OK";
    }
    Particle.publish("GPS", t.readLatLon(), 60, PRIVATE);
    Serial.println(t.readLatLon());
  } else {
    Serial.println('Waiting for GPS');
    messageGPSBlynk = "Waiting for GPS";
  }
  writeGPSInfoToBlynk(lat, lon, messageGPSBlynk);
  if (millis() - msLastSample >= msSAMPLE_INTERVAL){

  }
  t.updateGPS();
  if (millis()-lastPublish > delayMinutes*60*1000) {
    lastPublish = millis();
    Serial.println(t.preNMEA());
    if (t.gpsFix()) {
      if (transmittingData) {
        Particle.publish("location", t.readLatLon(), 60, PRIVATE);
      }
      Serial.println(t.readLatLon());
      client.publish("homeassistant/fin/gps",String(t.readLatLon()));

    }
  }
}

void getTemp(){
  float _temp;
  int   i = 0;
  do {
    _temp = ds18b20.getTemperature();
  }
  while (!ds18b20.crcCheck() && MAXRETRY > i++);
  if (i < MAXRETRY) {
    celsius = _temp;
    fahrenheit = ds18b20.convertToFahrenheit(_temp);
  }
  else {
    celsius = fahrenheit = NAN;
    Serial.println("Invalid reading");
  }
  Serial.println(celsius);
  client.publish("homeassistant/fin/ds18b20",String(celsius));
  Serial.println(fahrenheit);
  writeTemperatureToBlynk(celsius, fahrenheit);

  delay(5000);
  msLastSample = millis();
}

void displayOled(){
  if(isnan(celsius) || isnan(fahrenheit)) {
    oled.clearDisplay();
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(5, 0);
    oled.print('Temp Error!');
    return;
  }
  oled.clearDisplay();
  oled.setTextColor(WHITE);
  oled.setTextSize(2.5);
  oled.setCursor(5, 15);
  oled.print(celsius);
  oled.print(" C");
  oled.setCursor(5, 35);
  oled.print(fahrenheit);
  oled.print(" F");
}

void writeTemperatureToBlynk(double tempC, double tempF) {
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_TEMPC, tempC);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_TEMPH, tempF);
}

void writeGPSInfoToBlynk(float lattitude, float longitude, String gpsMessage) {
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_LAT, lattitude);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_LON, longitude);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_GPSMSG, gpsMessage);
}

void writeDeviceInfoToBlynk(float battery_voltage, float battery_percentage) {
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_BATTVOLTAGE, battery_voltage);
  Blynk.virtualWrite(BLYNK_VIRTUAL_PIN_BATTPERCENTAGE, battery_percentage);
}

BLYNK_WRITE(BLYNK_VIRTUAL_PIN_ZEBRA) {
        int r = param[0].asInt();
        int g = param[1].asInt();
        int b = param[2].asInt();
        if (r > 0 || g > 0 || b > 0) {
                RGB.control(true);
                RGB.color(r, g, b);
        } else {
                RGB.control(false);
        }
}

// Attach a Button widget (mode: Push) to the Virtual pin 13 - and send sweet tweets!
BLYNK_WRITE(BLYNK_VIRTUAL_PIN_TWITTER) {
        if (param.asInt() == 1) { // On button down...
                // Tweeting!
                // Note:
                //   We allow 1 tweet per minute for now.
                //   Twitter doesn't allow identical subsequent messages.
                Blynk.tweet("pingping is Online!");

                // Pushing notification to the app!
                // Note:
                //   We allow 1 notification per minute for now.
                Blynk.notify("Currently Streaming Data!");
        }
}

BLYNK_WRITE(BLYNK_VIRTUAL_PIN_SLIDE_TEMP_INPUT) {
}
