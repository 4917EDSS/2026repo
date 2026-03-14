/*
TEAM 4917 SIR LANCERBOT 2026

Download the Adafruit board package for running the RP2040 - it allows us to 
command the GPIO of the RP2040 directly while it thinks that it is an 
Arduin Nano RP2040 connect. (If it can dream it, it can be it)
*/

#include "Adafruit_VL53L0X.h"
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

Adafruit_VL53L0X range_sensor = Adafruit_VL53L0X();

int xshut0 = 18;
int xshut1 = 19;
int xshut2 = 17;
int xshut3 = 16;

void sensorInit(bool sensor0, bool sensor1, bool sensor2, bool sensor3) {
  // Sensor status - default to true for each sensor because they might not be all connected
  bool stat0 = true;
  bool stat1 = true;
  bool stat2 = true;
  bool stat3 = true;

  int xshutDelay = 300;
  int sensorBootAllocation = 400;
  
  if (sensor0) {
    digitalWrite(xshut0, LOW);
    digitalWrite(xshut1, LOW);
    digitalWrite(xshut2, LOW);
    digitalWrite(xshut3, LOW);
    delay(xshutDelay);
    digitalWrite(xshut0, HIGH);

    // Allow some time for the sensor to start up
    delay(sensorBootAllocation);
    // Initialize range sensor driver
    if (!range_sensor.begin()) {
      Serial.print("Failed to boot VL53L0X. SENSOR 0!! Stopping.\n");
      stat0 = false;
    }
  } 

  if (sensor1) {
    digitalWrite(xshut0, LOW);
    digitalWrite(xshut1, LOW);
    digitalWrite(xshut2, LOW);
    digitalWrite(xshut3, LOW);
    delay(xshutDelay);
    digitalWrite(xshut1, HIGH);

    // Allow some time for the sensor to start up
    delay(sensorBootAllocation);
    // Initialize range sensor driver
    if (!range_sensor.begin()) {
      Serial.print("Failed to boot VL53L0X. SENSOR 1!! Stopping.\n");
      stat1 = false;
    }
  } 

  if (sensor2) {
    digitalWrite(xshut0, LOW);
    digitalWrite(xshut1, LOW);
    digitalWrite(xshut2, LOW);
    digitalWrite(xshut3, LOW);
    delay(xshutDelay);
    digitalWrite(xshut2, HIGH);

    // Allow some time for the sensor to start up
    delay(sensorBootAllocation);
    // Initialize range sensor driver
    if (!range_sensor.begin()) {
      Serial.print("Failed to boot VL53L0X. SENSOR 2!!! Stopping.\n");
      stat2 = false;
    }
  }

  if (sensor3) {
    digitalWrite(xshut0, LOW);
    digitalWrite(xshut1, LOW);
    digitalWrite(xshut2, LOW);
    digitalWrite(xshut3, LOW);
    delay(xshutDelay);
    digitalWrite(xshut3, HIGH);

    // Allow some time for the sensor to start up
    delay(sensorBootAllocation);
    // Initialize range sensor driver
    if (!range_sensor.begin()) {
      Serial.print("Failed to boot VL53L0X. SENSOR 3!!! Stopping.\n");
      stat3 = false;
    }
  }

  // One of the sensors is clapped, stop the initialization process
  if (!(stat0 && stat1 && stat2 && stat3)) {
    // Display shenanigans 
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.write("SENSORS CLAPPED!!!");
    display.display();

    while(1);
  }

}

void setup() {
  // Give the OLED time to boot
  delay(1000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Sensors
  pinMode(xshut0, OUTPUT);
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);

  /*
  digitalWrite(xshut0, LOW);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  delay(300);
  digitalWrite(xshut0, HIGH);
  */

  sensorInit(true, false, false, false);

  // start serial moniter for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; //wait for serial port to connect. Needed for native USB port only
  }
  
  /*
  // Initialize range sensor driver
  if (!range_sensor.begin()) {
    Serial.print("Failed to boot VL53L0X. Stopping.\n");

    // Display shenanigans 
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.write("SENSORS CLAPPED!!!");
    display.display();

    while(1);

  } else {
    Serial.print("works??");
  }
  */

  /*
  // Initialize range sensor driver
  if (!range_sensor.begin()) {
    Serial.print("Failed to boot VL53L0X. SENSOR 0!! Stopping.\n");
    //stat0 = false;

    while(1);
  }
  */

  //sensorInit(true, false, false, false);

  Serial.print("VL53L0X driver loaded.\n");
  range_sensor.startRangeContinuous(); 
  

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
}

void loop() {
  char buffer[100];
  int distance;
  int angle;

  
  // Check if the range sensor has compleated a range mesurement
  if (range_sensor.isRangeComplete()){

    // Get the intager range value in mm
    distance = range_sensor.readRange();

    
    //Serial.print(0); // To freeze the lower limit
    //Serial.print(" ");
    //Serial.print(9000); // To freeze the upper limit
    //Serial.print(" ");
    

    // Debug serial print 
    sprintf (buffer, "Distance 1: %dmm\n",distance);
    //Serial.print(buffer);

    // Divide the range by 8 (bit shift 2^3 times to the right)
    angle = distance >> 3;
  }
  

  // Display shenanigans 
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  //sprintf (buffer, "Yo mama was a \nsnowblower");

  display.write(buffer);

  display.display();
}
