#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int redLed = 1;
const int adc0 = A0;   
const int adc1 = A1;
unsigned long currentTime;


void setup() {
  delay(1000);

  Serial.begin(9600);
  pinMode(redLed, OUTPUT);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.display();
  display.clearDisplay();

}

void loop() {
  currentTime = millis();
  display.clearDisplay();
  drawBar(sensorData());
  display.display();

  blinkLed(500);

  //Serial.print("Sensor 0: ");
  //Serial.println(sensorData());
} 

void blinkLed(int speed) {
  static unsigned long lastTime = 0;
  static bool flipflop = true;
  
  if((currentTime - lastTime) >= speed) {
    if(flipflop) {
      digitalWrite(redLed, HIGH);
      Serial.println("Light ON");
      flipflop = false;

    } else {
      digitalWrite(redLed, LOW);
      Serial.println("Light OFF");
      flipflop = true;
    }
    
    lastTime = currentTime;
  }  
}

int sensorData() {
  int sensor0 = analogRead(adc0);
  return sensor0;
}

void drawBar(int sensor) {
  display.fillRect(0, 0, sensor/7, 32, SSD1306_WHITE);
  display.fillRect(SCREEN_WIDTH/4, SCREEN_HEIGHT/4, (SCREEN_WIDTH/4)*2, (SCREEN_HEIGHT/4)*2, SSD1306_BLACK);

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor((SCREEN_WIDTH/4)+4, (SCREEN_HEIGHT/4)+4);
  display.cp437(true);
  char output[255];
  sprintf(output, "ADC0: %d", sensor);
  display.write(output);
}

















