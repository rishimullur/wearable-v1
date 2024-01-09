#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VEML7700.h"


/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_VEML7700 veml = Adafruit_VEML7700();

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// Variables for the low-pass filter
float temperatureFiltered = 0.0;
const float filterCoefficient = 0.2;

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  pinMode(6, OUTPUT);


    while (!Serial) { delay(10); }

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

  // == OPTIONAL =====
  // Can set non-default gain and integration time to
  // adjust for different lighting conditions.
  // =================
  // veml.setGain(VEML7700_GAIN_1_8);
  // veml.setIntegrationTime(VEML7700_IT_100MS);

  // Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  // Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
   
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
   // init done
  display.display();
  delay(100);
  display.clearDisplay();
  display.display();
  display.setTextSize(1.2);
  display.setTextColor(WHITE);
  
  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  delayTime = 1000;

  Serial.println();
}


void loop() { 
  
  display.setCursor(0,0);
  display.clearDisplay();

  // Serial.print("raw ALS: "); Serial.println(veml.readALS());
  // Serial.print("raw white: "); Serial.println(veml.readWhite());
  // Serial.print("lux: "); Serial.println(veml.readLux());

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold");

  }
  
  //  // Read the raw temperature from the sensor
  // float temperatureRaw = bme.readTemperature();

  // // Apply the low-pass filter
  // temperatureFiltered = (filterCoefficient * temperatureRaw) + ((1 - filterCoefficient) * temperatureFiltered);

  // Print the filtered temperature to the Serial output
  // Serial.print("Filtered Temperature = ");
  // Serial.println(temperatureFiltered);

  // Serial.print("Temperature = "); Serial.print(bme.readTemperature()); Serial.println(" *C");
  display.print("Temperature: "); display.print(bme.readTemperature()); display.println(" *C");

  // Serial.print("Pressure = "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");
  display.print("Pressure: "); display.print(bme.readPressure() / 100.0F); display.println(" hPa");

  // Serial.print("Humidity = "); Serial.print(bme.readHumidity()); Serial.println(" %");
  display.print("Humidity: "); display.print(bme.readHumidity()); display.println(" %");

  // Serial.print("lux = "); Serial.print(veml.readLux());
  //  Serial.println(" lx");
  display.print("Lux: "); display.print(veml.readLux()); display.println(" lx");

  // Serial.print("T:");
  // Serial.println(bme.readTemperature());

    // Read the raw lux value from the VEML7700
  float vemlLux = veml.readLux();

  // Apply the offset
  float correctedLux = vemlLux - 0.92;

  // Print the corrected lux value to the Serial output
  Serial.print("Corrected Lux = ");
  Serial.println(correctedLux);


  if (veml.readLux() < 50) {
  //     digitalWrite(2, HIGH);
  //   // Add code here to send A2Voltage using your preferred communication method (e.g., Serial.write())
  // Serial.print("lux low ");
  digitalWrite(6, HIGH);   
  delay(1000); 
  digitalWrite(6, LOW);              
  }

  Serial.println();
  display.display();
  delay(1000);
}