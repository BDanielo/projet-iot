#include <Arduino.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <MQUnifiedsensor.h>

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


/************************ Hardware Related Macros ****************************/
#define Board                   ("ESP32")
#define Pin                     (34)  // Analog input pin connected to MQ-2
#define Type                    ("MQ-2") // Sensor type
#define Voltage_Resolution      (3.3) // ESP32 ADC voltage resolution
#define ADC_Bit_Resolution      (12)  // ESP32 ADC bit resolution
#define RatioMQ2CleanAir        (9.83) // RS/R0 ratio in clean air from datasheet

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

void setup() {
  Serial.begin(9600);

  delay(1000); // Allow system to stabilize

  Serial.println("Initializing hardware version hw486...");
  dht.setup(DHT_PIN, DHTesp::DHT22); // Connect DHT sensor to DHT_PIN

  // Initialize the RGB LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Initialize the photo sensor pin as input
  pinMode(photoSensorPin, INPUT);

  // Initialize the MQ2 sensor
  MQ2.setRegressionMethod(1); // Set regression method to linear
  MQ2.setA(36974); // Set A value for MQ2
  MQ2.setB(-3.109); // Set B value for MQ2
  // MQ2.init();

  Serial.print("Calibrating MQ2 sensor...");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update(); // Update readings
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
    delay(1000);
  }
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
}

void loop() {
  delay(dht.getMinimumSamplingPeriod());
  MQ2.update();

  float LPG = MQ2.readSensor(); // Read LPG concentration in ppm

  Serial.print("LPG concentration: ");
  Serial.print(LPG);
  Serial.println(" ppm");

  float temperature = dht.getTemperature();
  float humidity = dht.getHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.println("Humidity: " + String(humidity) + " %");
  Serial.print("Temperature: " + String(temperature) + " °C\n");

  int redValue = map(humidity, 50, 100, 0, 255);
  int greenValue = map(humidity, 50, 100, 255, 0);
  int blueValue = 0; 

  // Set the color of the RGB LED
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);

  // Read the value from the photo sensor
  int photoSensorValue = analogRead(photoSensorPin);
  Serial.println("Photo sensor value: " + String(map(photoSensorValue, 2000, 0, 0, 100)) + " %");
}