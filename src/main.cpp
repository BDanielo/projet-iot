#include <Arduino.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <MQUnifiedsensor.h>
#include <Wifi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// Wifi parameters
const char* ssid = "Xiaomi11TPro";
const char* password = "1234567890";

// Define the DHT sensor type and the data pin
#define DHT_PIN 26

DHTesp dht;

// Define the pins for the RGB LED
const int redPin = 32;
const int greenPin = 25;
const int bluePin = 33;

// Define the pin for the photo sensor
const int photoSensorPin = 35;

// Define the pin for the MQ2 sensor
#define MQ2_PIN 34
#define BUZZER_PIN 27

/************************ Hardware Related Macros ****************************/
#define Board                   ("ESP32")
#define Pin                     (34)  // Analog input pin connected to MQ-2
#define Type                    ("MQ-2") // Sensor type
#define Voltage_Resolution      (3.3) // ESP32 ADC voltage resolution
#define ADC_Bit_Resolution      (12)  // ESP32 ADC bit resolution
#define RatioMQ2CleanAir        (9.83) // RS/R0 ratio in clean air from datasheet

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

WebServer server(80); // Start the server on port 80

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_user = ""; // Not required for public brokers
const char* mqtt_password = ""; // Not required for public brokers

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
unsigned long interval;

int LedColorRed = 0;
int LedColorGreen = 0;
int LedColorBlue = 0;

char dht22Values[32];
char mq2Values[32];

unsigned long coValue = 0;

boolean isOverrideLed = false;

// State variables for non-blocking timing
enum State { INIT, CALIBRATING_MQ2, WIFI_CONNECTING, MQTT_CONNECTING, RUNNING };
State currentState = INIT;

float calcR0 = 0;
int calibrationStep = 0;
unsigned long lastCalibrationTime = 0;

boolean wifiConnecting = false;
unsigned long lastWifiCheck = 0;

unsigned long lastMqttAttempt = 0;

void setUpMQ2_Init() {
  // Initialize the MQ2 sensor
  MQ2.setRegressionMethod(1); // Set regression method to linear
  MQ2.setA(36974); // Set A value for MQ2
  MQ2.setB(-3.109); // Set B value for MQ2

  Serial.print("Calibrating MQ2 sensor...");

  // Initialize variables for calibration
  calcR0 = 0;
  calibrationStep = 0;
  lastCalibrationTime = millis();
}

boolean setUpMQ2_Calibrate() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCalibrationTime >= 1000) {
    lastCalibrationTime = currentMillis;
    MQ2.update(); // Update readings
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");

    calibrationStep++;

    if (calibrationStep >= 10) {
      MQ2.setR0(calcR0 / 10); // Set the average R0
      Serial.println(" done!");

      if (isnan(MQ2.getR0())) {
        Serial.println("Error: R0 is NaN, check your wiring and connections.");
        while (1);
      }

      Serial.print("Calibration completed. R0 = ");
      Serial.println(MQ2.getR0());

      // Allow time for the sensor to heat up and stabilize
      MQ2.init();
      return true; // Calibration is complete
    }
  }
  return false; // Calibration is not complete yet
}

void readPhotoSensorValues() {
  // Read the value from the photo sensor
  int photoSensorValue = analogRead(photoSensorPin);
  Serial.println("Photo sensor value: " + String(map(photoSensorValue, 2000, 0, 0, 100)) + " %");
  client.publish("danielo/lumen", String(photoSensorValue).c_str());
}

int readDHT22Values() {
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }

  Serial.println("Humidity: " + String(humidity) + " %");
  Serial.print("Temperature: " + String(temperature) + " °C\n");

  return humidity;
}

void setLedRGBColor(int humidity) {
  if (!isOverrideLed)
  {
    LedColorRed = map(humidity, 50, 100, 0, 255);
    LedColorGreen = map(humidity, 50, 100, 255, 0);
    LedColorBlue = 0; 

    // Set the color of the RGB LED
    analogWrite(redPin, LedColorRed);
    analogWrite(greenPin, LedColorGreen);
    analogWrite(bluePin, LedColorBlue);
  }
}

void readMQ2Values() {
  MQ2.update();

  coValue = MQ2.readSensor(); // Read LPG concentration in ppm

  Serial.print("CO concentration: ");
  Serial.print(coValue);
  Serial.println(" ppm (parts per million)");

  if (coValue < 10) {
    analogWrite(BUZZER_PIN, 5);
  } else {
    analogWrite(BUZZER_PIN, LOW);
  }

  client.publish("danielo/co", String(coValue).c_str());
}

boolean wifiConnect_NonBlocking() {
  unsigned long currentMillis = millis();

  if (!wifiConnecting) {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    wifiConnecting = true;
    lastWifiCheck = currentMillis;
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Print local IP address
    Serial.println("\nConnected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true; // WiFi connected
  } else {
    if (currentMillis - lastWifiCheck >= 500) {
      lastWifiCheck = currentMillis;
      Serial.print(".");
    }
    return false; // WiFi not connected yet
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<title>Données capteurs</title>";
  html += "<style> body { font-family: Arial, sans-serif; }";
  html += "h1 { color: #333; }";
  html += "p { color: #666; }";
  html += "input[type=\"number\"] { width: 50px; background-color: #f9f9f9; border: 1px solid #ccc; padding: 5px; }";
  html += "input[type=\"submit\"] { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; cursor: pointer; }";
  html += "input[type=\"submit\"]:hover { background-color: #45a049; }";
  html += "form { margin-top: 20px; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Données capteurs</h1>";
  html += "<p>Température: <span id=\"temperature\">Chargement...</span> &deg;C</p>";
  html += "<p>Humidité: <span id=\"humidity\">Chargement...</span> %</p>";
  html += "<p>MQ2 valeur CO: <span id=\"mq2Value\">Chargement...</span> ppm</p>";
  html += "<h1>LED</h1>";
  html += "<form action=\"/led\" method=\"get\">";
  html += "Rouge: <input type=\"number\" name=\"red\" min=\"0\" max=\"255\"><br>";
  html += "Vert: <input type=\"number\" name=\"green\" min=\"0\" max=\"255\"><br>";
  html += "<input type=\"submit\" value=\"Appliquer\">";
  html += "</form>";
  html += "<script>";
  html += "function fetchSensorData() {";
  html += "  fetch('/sensor').then(response => response.json()).then(data => {";
  html += "    document.getElementById('temperature').innerText = data.temperature;";
  html += "    document.getElementById('humidity').innerText = data.humidity;";
  html += "    document.getElementById('mq2Value').innerText = data.mq2Value;";
  html += "  });";
  html += "}";
  html += "setInterval(fetchSensorData, 2000);";
  html += "</script>";
  html += "</body></html>";

  // Send the page to the client
  server.send(200, "text/html", html);
}

void handleSensorData() {
  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();
  float mq2Value = MQ2.readSensor();

  String json = "{";
  json += "\"temperature\":";
  if (isnan(temperature)) {
    json += "\"Erreur\"";
  } else {
    json += String(temperature);
  }
  json += ",";

  json += "\"humidity\":";
  if (isnan(humidity)) {
    json += "\"Erreur\"";
  } else {
    json += String(humidity);
  }
  json += ",";

  json += "\"mq2Value\":";
  if (isnan(mq2Value)) {
    json += "\"Erreur\"";
  } else {
    json += String(mq2Value);
  }
  json += "}";
  server.send(200, "application/json", json);
}

void handleLedChange() {
  if (server.hasArg("red") && server.hasArg("green")) {
    isOverrideLed = true;
    int redValue = server.arg("red").toInt();
    int greenValue = server.arg("green").toInt();
    Serial.printf("Setting LED color to R:%d G:%d\n", redValue, greenValue);
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    server.send(200, "text/plain", "Valeur de la LED modifiée");
  } else {
    server.send(400, "text/plain", "Paramètres red ou green manquants");
        isOverrideLed = false;

  }
}

void initServer() {
  server.on("/", handleRoot);
  server.on("/led", handleLedChange);
  server.on("/sensor", handleSensorData);
  server.begin();
  Serial.println("Serveur HTTP démarré");
}

boolean mqttConnect_NonBlocking() {
  unsigned long currentMillis = millis();

  if (client.connected()) {
    return true; // MQTT connected
  } else {
    if (currentMillis - lastMqttAttempt >= 2000) {
      lastMqttAttempt = currentMillis;
      Serial.print("Connecting to MQTT...");
      if (client.connect("ESP8266ClientDanielo", mqtt_user, mqtt_password)) {
        Serial.println("connected");
        client.subscribe("danielo/buzzer");
        client.subscribe("danielo/led");
        return true; // MQTT connected
      } else {
        Serial.print("failed with state ");
        Serial.println(client.state());
      }
    }
    return false; // MQTT not connected yet
  }
}

void getTempHumidity() {
  if (dht.getStatus() != 0) {
    Serial.println("Erreur lors de la lecture du capteur DHT22");
    snprintf(dht22Values, sizeof(dht22Values), "Erreur DHT22");
    return;
  } else {
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    snprintf(dht22Values, sizeof(dht22Values), "%.1fC %.1f%%", temperature, humidity);

    client.publish("danielo/temperature", String(temperature).c_str());
    client.publish("danielo/humidity", String(humidity).c_str());
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message reçu sur le topic [");
  Serial.print(topic);
  Serial.print("] : ");
  String message;

  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Vérifier le contenu du message pour déclencher le buzzer
  if (String(topic) == "danielo/buzzer") {
    if (message == "ON") {
      // Activer le buzzer
      analogWrite(BUZZER_PIN, 5);
      Serial.println("Buzzer activé");
    } else if (message == "OFF") {
      // Désactiver le buzzer
      analogWrite(BUZZER_PIN, 0);
      Serial.println("Buzzer désactivé");
    }
  }

  if (String(topic) == "danielo/led") {
    isOverrideLed = true;
    int r, g, b;
    if (sscanf(message.c_str(), "rgb(%d, %d, %d)", &r, &g, &b) == 3) {
      analogWrite(redPin, r);
      analogWrite(greenPin, g);
      analogWrite(bluePin, b);
      Serial.printf("LED color set to R:%d G:%d B:%d\n", r, g, b);
    } else {
      Serial.println("Invalid RGB format");
    }
  } else {
    isOverrideLed = false;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing hardware version hw486...");

  // Initialize DHT sensor
  dht.setup(DHT_PIN, DHTesp::DHT22); // Connect DHT sensor to DHT_PIN

  // Initialize the RGB LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize the photo sensor pin as input
  pinMode(photoSensorPin, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize variables
  currentState = INIT;

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  unsigned long currentMillis = millis();
  server.handleClient();
  client.loop();

  switch (currentState) {
    case INIT:
      setUpMQ2_Init();
      currentState = CALIBRATING_MQ2;
      break;

    case CALIBRATING_MQ2:
      if (setUpMQ2_Calibrate()) {
        currentState = WIFI_CONNECTING;
      }
      break;

    case WIFI_CONNECTING:
      if (wifiConnect_NonBlocking()) {
        currentState = MQTT_CONNECTING;
        initServer(); // Start server after WiFi is connected
      }
      break;

    case MQTT_CONNECTING:
      if (mqttConnect_NonBlocking()) {
        interval = dht.getMinimumSamplingPeriod();
        previousMillis = currentMillis;
        currentState = RUNNING;
      }
      break;

    case RUNNING:
      if (!client.connected()) {
        currentState = MQTT_CONNECTING;
        break;
      }

      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Serial.println("\n\n\n\n\n=====================================");
        readMQ2Values();
        readPhotoSensorValues();
        int humidity = readDHT22Values();
        setLedRGBColor(humidity);
        getTempHumidity();
        Serial.println("=====================================");
      }
      break;
  }
}
