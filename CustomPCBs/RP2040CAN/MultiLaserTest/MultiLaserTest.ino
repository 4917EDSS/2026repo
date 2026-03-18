/*
TEAM 4917 SIR LANCERBOT 2026

- Add additional board manager URL (Preferences): https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
- Download the Earle F Philihower Raspberry Pi Pico/RP2040/RP2350 board manager version 5.5.1.
- Set board to Arduino Nano RP2040 Connect

It allows us to command the GPIO of the RP2040 directly while it thinks that it is an 
Arduino Nano RP2040 connect. (If it can dream it, it can be it)

- Add library Adafruit_VL53L0X version 1.2.5 and dependencies
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

Adafruit_VL53L0X range_sensor0 = Adafruit_VL53L0X();
Adafruit_VL53L0X range_sensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X range_sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X range_sensor3 = Adafruit_VL53L0X();

int xshut0 = 18;
int xshut1 = 19;
int xshut2 = 17;
int xshut3 = 16;

// Please don't touch this (Squint and you can see the logo)
static const unsigned char PROGMEM lancerbot_logo_bmp[] = {
0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00000001,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00010000,0B11000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00010111,0B00111100,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00001110,0B00000010,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00000111,0B01100010,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00000010,0B10010100,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00000011,0B00001000,0B00000000,0B00000000,0B00000000,0B00000000,0B00110001,0B11110000,0B11001111,0B11100000,
0B00000000,0B00000011,0B10011000,0B00000000,0B00000000,0B00000000,0B00000000,0B01110011,0B11111001,0B11001111,0B11100000,
0B00000000,0B00000110,0B11000100,0B00000000,0B00000000,0B00000000,0B00000000,0B11110011,0B10011111,0B11001111,0B11100000,
0B00000000,0B01111010,0B10101000,0B00000000,0B00000000,0B00000000,0B00000001,0B11110010,0B00001111,0B11001100,0B11100000,
0B00001111,0B11111110,0B01001000,0B00000000,0B00000000,0B00000000,0B00000011,0B10110011,0B00001100,0B11000000,0B11100000,
0B00010000,0B00111111,0B11110000,0B00000000,0B00001011,0B00010000,0B00000011,0B00110011,0B10011100,0B11000000,0B11100000,
0B00100100,0B00010111,0B01000000,0B00000110,0B00000011,0B00100110,0B00000011,0B11111001,0B11111100,0B11000001,0B11000000,
0B00100110,0B00000111,0B11000000,0B00000011,0B00111101,0B11111001,0B00000011,0B11111000,0B00011100,0B11000001,0B11000000,
0B00101110,0B01110001,0B11101000,0B01100001,0B11100111,0B11111000,0B01000011,0B11111011,0B10011100,0B11000001,0B11000000,
0B00101101,0B01001001,0B11101101,0B10000111,0B00111111,0B11111111,0B10100000,0B00110011,0B10011100,0B11000011,0B10000000,
0B01010111,0B10001001,0B11101100,0B00010001,0B11111111,0B10111111,0B10010000,0B01111011,0B11111111,0B11100011,0B10000000,
0B01110010,0B01100111,0B11101110,0B01111001,0B11110111,0B00111111,0B11000000,0B01111001,0B11111011,0B11100011,0B10000000,
0B01000000,0B11011001,0B11111111,0B10001111,0B11101111,0B11011111,0B10110000,0B00000000,0B00000000,0B00000000,0B00000000,
0B01000000,0B01100011,0B11110000,0B00000111,0B11011111,0B10110000,0B01111111,0B11111111,0B11111111,0B11111111,0B11000000,
0B01011100,0B00000111,0B11111111,0B00111111,0B11011101,0B01000000,0B10111111,0B11110000,0B00000000,0B00000000,0B00000000,
0B01101101,0B10001000,0B00000011,0B10000000,0B00000000,0B10000000,0B00110000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00100110,0B10101000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00100000,0B01010011,0B11000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00010000,0B00100100,0B00100000,0B00001000,0B00001000,0B10001001,0B11100111,0B11011110,0B01111000,0B11100111,0B11000000,
0B00001000,0B11000011,0B00000000,0B00001000,0B00011100,0B11001010,0B00010100,0B00010001,0B01000101,0B00010001,0B00000000,
0B00000111,0B00000000,0B11001011,0B10001000,0B00010100,0B10101010,0B00000111,0B10010001,0B01111001,0B00010001,0B00000000,
0B00000000,0B00000100,0B00101010,0B10001000,0B00100010,0B10011010,0B00010100,0B00011110,0B01000101,0B00010001,0B00000000,
0B00000000,0B00000011,0B11001010,0B00001111,0B10100010,0B10001001,0B11100111,0B11010011,0B01111000,0B11100001,0B00000000,
0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000
};


void sensorInit(bool sensor0, bool sensor1, bool sensor2, bool sensor3) {
  // Sensor status - default to true for each sensor because they might not be all connected
  bool stat0 = true;
  bool stat1 = true;
  bool stat2 = true;
  bool stat3 = true;
  
  // Set all of the reset pins low
  digitalWrite(xshut0, LOW);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);
  
  // Initialize each of the sensors individually, ONLY IF WE HAVE THEM ISNTALLED!! (Otherwise we initialize sensors that don't exist)
  if (sensor0) {
    // Call the initialization function
    stat0 = initializeRangeSensor(range_sensor0, 0x31, xshut0);
  }

  if (sensor1) {
    stat1 = initializeRangeSensor(range_sensor1, 0x32, xshut1);
  }

  if (sensor2) {
    stat2 = initializeRangeSensor(range_sensor2, 0x33, xshut2);
  }

  if (sensor3) {
    stat3 = initializeRangeSensor(range_sensor3, 0x34, xshut3);
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

  // Start the sensors that we need to intitialize 
  if (sensor0) {
    range_sensor0.startRangeContinuous();
  }
  
  if (sensor1) {
    range_sensor1.startRangeContinuous();
  }

  if(sensor2) {
    range_sensor2.startRangeContinuous();
  }

  if(sensor3) {
    range_sensor3.startRangeContinuous();
  }
}


// A function that starts up the individual range sensor 
bool initializeRangeSensor(Adafruit_VL53L0X &sensor, uint8_t address, int xshut) {
  int xshutDelay = 300;
  int sensorBootAllocation = 400;

  delay(xshutDelay);
  digitalWrite(xshut, HIGH);

  // Allow some time for the sensor to start up
  delay(sensorBootAllocation);
  // Initialize range sensor driver
  if (!sensor.begin()) {
    Serial.print("Failed to boot VL53L0X. Stopping.\n");
    // Bad, sensor failed
    return false;
  }

  // Set the adress of each sensor so they can be accessed later
  sensor.setAddress(address);

  // success! 
  return true;
}


void setup() {
  // Give the OLED time to boot
  delay(1000);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  drawLogo();

  // Clear the buffer
  display.clearDisplay();

  
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

  sensorInit(true, true, true, true);

  // start serial moniter for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; //wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("VL53L0X driver loaded.\n");
}

void loop() {
  char buffer[100];
  static uint16_t distances[4];  // array of saved distances for each sensor
  int angle;
  bool updateDisplay = false;


  // CAN'T READ DATA FROM SENSORS THAT DON'T EXIST!!!

  // Check if the range sensor has compleated a range mesurement
  if (range_sensor0.isRangeComplete()){
    // Get the intager range value in mm
    distances[0] = range_sensor0.readRange();
    updateDisplay = true;

    //sprintf (buffer, "Distance 1: %dmm\nDistance 2: ",distance0);
    //sprintf (buffer, "Distance 1: %u", distance0);
  }

  // Check if the range sensor has compleated a range mesurement
  if (range_sensor1.isRangeComplete()){
    // Get the intager range value in mm
    distances[1] = range_sensor1.readRange();
    updateDisplay = true;
  }

  if (range_sensor2.isRangeComplete()) {
    distances[2] = range_sensor2.readRange();
    updateDisplay = true;
  }

  if (range_sensor3.isRangeComplete()) {
    distances[3] = range_sensor3.readRange();
    updateDisplay = true;
  }

  // Only update the display if there is sensor data to display 
  if (updateDisplay) {
    sprintf(buffer, "Sensor 1: %umm\nSensor 2: %umm\nSensor 3: %umm\nSensor 4: %umm", distances[0], distances[1], distances[2], distances[3]);

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
}

// Draw the lancerbot logo!!
void drawLogo(void) {
  display.clearDisplay();

  display.drawBitmap(
    //(display.width()  - LOGO_WIDTH ) / 2,
    //(display.height() - LOGO_HEIGHT) / 2,
    (display.width() - 88) / 2,
    0,
    lancerbot_logo_bmp, 88, 32, 1);
  display.display();
  delay(1000);
}
