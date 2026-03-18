/*
TEAM 4917 SIR LANCERBOT 2026

- Add additional board manager URL (Preferences): https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
- Download the Earle F Philihower Raspberry Pi Pico/RP2040/RP2350 board manager version 5.5.1.
- Set board to Arduino Nano RP2040 Connect

It allows us to command the GPIO of the RP2040 directly while it thinks that it is an 
Arduino Nano RP2040 connect. (If it can dream it, it can be it)

- Add library Adafruit_VL53L0X version 1.2.5 and dependencies
*/

#include "frc_mcp2515.h"
#include "frc_CAN.h"
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

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

// Initialize sensors
Adafruit_VL53L0X rangeSensors[4] = {Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X()};

// Define hardware locations
const int xshuts[4] = {18, 19, 17, 16};
const int greenLed = 0;
const int redLed = 1;

#define CAN_CS 11
#define CAN_INTERRUPT 2     // Appears to be missing on this board D:
#define CAN_PERIOD_MS 100
#define CAN_PACKET_SIZE 8
/* 
ID 1 for Shooter
ID 2 for Hopper Empty
ID 3 for Hopper Full    ### ToF BOARD ###
ID 4 for Climb
*/
#define CAN_DEVICE_ID 1
#define CAN_DEVICE_API 0x123

int numSensorsOperating;

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

// Also don't touch this one. This displays when there is a critical hardware error with one of the sensors 
static const unsigned char PROGMEM clapped_warning_bmp[] = {
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B00000011,0B11110000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B00000111,0B11111000,0B00000000,0B00011111,0B00000111,0B11111110,0B01110000,0B01110000,0B11111000,0B00000111,0B11100000,0B11111111,0B10000001,0B11110000,0B00000000,
  0B00000000,0B00001111,0B00111100,0B00000000,0B00111111,0B11000111,0B11111110,0B01111000,0B01110001,0B11111110,0B00001111,0B11110000,0B11111111,0B11000011,0B11111100,0B00000000,
  0B00000000,0B00011110,0B00011110,0B00000000,0B01111111,0B11000111,0B11111110,0B01111000,0B01110011,0B11111110,0B00011111,0B11111000,0B11111111,0B11100111,0B11111100,0B00000000,
  0B00000000,0B00011100,0B00001110,0B00000000,0B01110000,0B11100111,0B00000000,0B01111100,0B01110011,0B10000111,0B00111100,0B00111100,0B11100000,0B11100111,0B00001110,0B00000000,
  0B00000000,0B00111000,0B00000111,0B00000000,0B01110000,0B00000111,0B00000000,0B01111110,0B01110011,0B10000000,0B00111000,0B00011100,0B11100000,0B11100111,0B00000000,0B00000000,
  0B00000000,0B00111000,0B00000111,0B00000000,0B00111111,0B00000111,0B11111100,0B01111110,0B01110001,0B11111000,0B00111000,0B00011100,0B11111111,0B11000011,0B11110000,0B00000000,
  0B00000000,0B01110000,0B00000011,0B10000000,0B00011111,0B11000111,0B11111100,0B01110111,0B01110000,0B11111110,0B00111000,0B00001100,0B11111111,0B11000001,0B11111100,0B00000000,
  0B00000000,0B11110000,0B00000011,0B11000000,0B00000111,0B11100111,0B11111100,0B01110111,0B11110000,0B00111111,0B00111000,0B00011100,0B11111111,0B00000000,0B01111110,0B00000000,
  0B00000000,0B11100000,0B11000001,0B11000000,0B00000000,0B11100111,0B00000000,0B01110011,0B11110000,0B00000111,0B00111000,0B00011100,0B11100111,0B10000000,0B00001110,0B00000000,
  0B00000001,0B11000001,0B11100000,0B11100000,0B01110000,0B11100111,0B00000000,0B01110001,0B11110011,0B10000111,0B00111100,0B00111100,0B11100011,0B11000111,0B00001110,0B00000000,
  0B00000001,0B11000001,0B11100000,0B11100000,0B01111111,0B11100111,0B11111110,0B01110001,0B11110011,0B11111111,0B00011111,0B11111000,0B11100001,0B11000111,0B11111110,0B00000000,
  0B00000011,0B10000001,0B11100000,0B01110000,0B00111111,0B11000111,0B11111110,0B01110000,0B11110001,0B11111110,0B00001111,0B11110000,0B11100000,0B11100011,0B11111100,0B00000000,
  0B00000111,0B10000001,0B11100000,0B01110000,0B00011111,0B10000111,0B11111110,0B01110000,0B01110000,0B11111100,0B00000111,0B11100000,0B11100000,0B11110001,0B11111000,0B00000000,
  0B00000111,0B00000001,0B11100000,0B00111000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00001110,0B00000001,0B11100000,0B00111100,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00001110,0B00000000,0B11000000,0B00011100,0B00000000,0B00011111,0B00000111,0B00000000,0B00001110,0B00000111,0B11111000,0B01111111,0B10000111,0B11111110,0B01111111,0B10000000,
  0B00011100,0B00000000,0B11000000,0B00001110,0B00000000,0B00111111,0B11000111,0B00000000,0B00011110,0B00000111,0B11111100,0B01111111,0B11000111,0B11111110,0B01111111,0B11000000,
  0B00011100,0B00000000,0B11000000,0B00001110,0B00000000,0B01111111,0B11000111,0B00000000,0B00011111,0B00000111,0B11111110,0B01111111,0B11100111,0B11111110,0B01111111,0B11100000,
  0B00111000,0B00000000,0B11000000,0B00000111,0B00000000,0B11110001,0B11100111,0B00000000,0B00011111,0B00000111,0B00001110,0B01110000,0B11100111,0B00000000,0B01110000,0B11110000,
  0B01111000,0B00000000,0B00000000,0B00000111,0B10000000,0B11100000,0B11000111,0B00000000,0B00111011,0B10000111,0B00001110,0B01110000,0B11100111,0B00000000,0B01110000,0B01110000,
  0B01110000,0B00000000,0B00000000,0B00000011,0B10000000,0B11100000,0B00000111,0B00000000,0B00110011,0B10000111,0B11111110,0B01111111,0B11100111,0B11111100,0B01110000,0B01110000,
  0B11100000,0B00000000,0B11000000,0B00000001,0B11000000,0B11100000,0B00000111,0B00000000,0B00110001,0B10000111,0B11111100,0B01111111,0B11000111,0B11111100,0B01110000,0B01110000,
  0B11100000,0B00000001,0B11100000,0B00000001,0B11000000,0B11100000,0B00000111,0B00000000,0B01111111,0B11000111,0B11111000,0B01111111,0B10000111,0B11111100,0B01110000,0B01110000,
  0B11100000,0B00000000,0B11000000,0B00000001,0B11000000,0B11100000,0B11000111,0B00000000,0B01111111,0B11000111,0B00000000,0B01110000,0B00000111,0B00000000,0B01110000,0B01110000,
  0B11100000,0B00000000,0B00000000,0B00000001,0B11000000,0B11110001,0B11100111,0B00000000,0B11111111,0B11100111,0B00000000,0B01110000,0B00000111,0B00000000,0B01110000,0B11110000,
  0B11100000,0B00000000,0B00000000,0B00000001,0B11000000,0B01111111,0B11000111,0B11111100,0B11100000,0B11100111,0B00000000,0B01110000,0B00000111,0B11111110,0B01111111,0B11100000,
  0B01110000,0B00000000,0B00000000,0B00000011,0B10000000,0B00111111,0B11000111,0B11111100,0B11000000,0B01100111,0B00000000,0B01110000,0B00000111,0B11111110,0B01111111,0B11000000,
  0B01111111,0B11111111,0B11111111,0B11111111,0B10000000,0B00011111,0B00000111,0B11111101,0B11000000,0B01110111,0B00000000,0B01110000,0B00000111,0B11111110,0B01111111,0B10000000,
  0B00111111,0B11111111,0B11111111,0B11111111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00001111,0B11111111,0B11111111,0B11111100,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000
};

// Global variables
unsigned long long lastSendMs = 0; // track how long since we last sent a CAN packet

// Create an MCP2515 device. Only need to create 1 of these
frc::MCP2515 mcp2515{CAN_CS};

// Create an FRC CAN Device. You can create up to 16 of these in 1 progam
// Any more will overflow a global array.  The device number, manufacturer 
// and devicetype are basically filters for all incoming data.  You will 
// receive all callbacks associated with these and further process it
// based on the API ID and data packets.
frc::CAN frcCANDevice{CAN_DEVICE_ID, frc::CANManufacturer::kTeamUse, frc::CANDeviceType::kMiscellaneous};


// Callback function. This will be called any time a new message is received
// Matching one of the enabled devices.
void CANCallback(frc::CAN* can, int apiId, bool rtr, const frc::CANData& data) {
  // Put recieved code here
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback(uint32_t id, const frc::CANData& data) {
  // Unknown message code here
}

// Initialize all specified sensors
void sensorInit(int numSensors) {
  // Sensor status - default to true for each sensor because they might not be all connected
  bool sensorStatus = true;
  bool sensorBroken[4] = {false, false, false, false};

  // Define the global variable with the number of sensors that we are operating with
  numSensorsOperating = numSensors;

  // I2C adress of the sensors 
  uint8_t adresses[4] = {0x31, 0x32, 0x33, 0x34};
  
  // Set all of the XSHUT reset pins low
  for(int i = 0; i < 4; i++) {
    digitalWrite(xshuts[i], LOW);
  }

  // Loop through all of the initialized sensors 
  for(int i = 0; i < numSensors; i++) {
    // Attempt to initialize the sensors
    if(!initializeRangeSensor(rangeSensors[i], adresses[i], xshuts[i])) {
      // If a sensor is clapped, stop! 
      sensorStatus = false;
      sensorBroken[i] = true;
      break;
    }
  }

  // One of the sensors is clapped, stop the initialization process
  if (!sensorStatus) {
    // Find the clapped sensor
    int clappedSensor;
    for(int i = 0; i < 4; i++) {
      // If we have found the false sensor in the array, save the index at which it was found
      if (sensorBroken[i]) {
        clappedSensor = i;
      }
    }
    
    // Flash LED to make it more obvoius
    while(true) {
      digitalWrite(redLed, HIGH);
      display.clearDisplay();
      // Display the clapped sensors warning
      drawClapped();

      delay(1000);
      
      digitalWrite(redLed, LOW);
      // Display the number that's broken
      display.clearDisplay();
      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.setCursor(0, 0);     // Start at top-left corner
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      
      char buffer[100];
      sprintf(buffer, "SENSOR %d", clappedSensor+1);
      
      display.write(buffer);
      display.display();

      delay(1000);
    }
    
    //while(1);
  }

  // Start the sensors that we need to intitialize (connected ones)
  for(int i = 0; i < numSensors; i++) {
    rangeSensors[i].startRangeContinuous();
  }
}


// A function that starts up the individual range sensor 
bool initializeRangeSensor(Adafruit_VL53L0X &sensor, uint8_t address, int xshut) {
  // Delays required by the hardware in miliseconds 
  int xshutDelay = 300;
  int sensorBootAllocation = 400;

  // Make sure the shutdown pin of the sensor is high.
  // Each of these sensors need to be powered one at a time, or it won't get a unique adress 
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
  delay(1500);

  // Intialize the SPI port
  mcp2515.test_init();

  // Begin serial
  Serial.begin(115200);


  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Display the logo while all the sensors boot up
  display.display();
  drawLogo();

  // Clear the buffer
  display.clearDisplay();


  // Initialize the hardware locations of the shutdown pins as outputs
  for(int i = 0; i < 4; i++) {
    pinMode(xshuts[i], OUTPUT);
  }
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);

  // Initialize a specific number of sensors (1 - 4)
  sensorInit(4);                                                                              // CHANGE THIS VALUE IF WE ARE NOT USING 4 SENSORS!! 

  // start serial moniter for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; //wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("VL53L0X driver loaded.\n");

  
  // Initialize the MCP2515. If any error values are set, initialization failed
  auto err = mcp2515.reset();

  // CAN rate must be 1000KBPS to work with the FRC Ecosystem
  // Clock rate must match clock rate of CAN Board.
  err = mcp2515.setBitrate(frc::CAN_1000KBPS, frc::CAN_CLOCK::MCP_16MHZ);

  // Set up to normal CAN mode to allow it to communicate to the RIO. If you
  // just want to monitor the bus, you can use listen only mode.
  err = mcp2515.setNormalMode();
  //err = mcp2515.setListenOnlyMode();

  // Prepare our interrupt pin
  pinMode(CAN_INTERRUPT, INPUT);
  
  // Set up FRC CAN to be able to use the CAN Impl and callbacks
  // Last parameter can be set to nullptr if unknown messages should be skipped
  frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, UnknownMessageCallback);

  // All CAN Devices must be added to the read list. Otherwise they will not be handled correctly.
  frcCANDevice.AddToReadList();

  // Initialization Complete 
}

void loop() {
  // Define the buffer for printing text
  char buffer[100];
  // Define an array of all the distances aquired from the sensors 
  static uint16_t distances[4] = {0,0,0,0};
  // Is there new data to display on the screen? 
  bool updateDisplay = false;

  // Check each of the range sensors for new data
  for (int i = 0; i < numSensorsOperating; i++) {
    // Check if the sensor has new data to send
    if(rangeSensors[i].isRangeComplete()) {
      // Get the integer range value in mm
      distances[i] = rangeSensors[i].readRange();
      // There is new data, it should be displayed 
      updateDisplay = true; 
    }
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
  

  //int16_t distance[4];
  uint8_t data[CAN_PACKET_SIZE];

  // Update must be called every loop in order to receive messages
  frc::CAN::Update();

  // Update all sensors as frequently as possible. This could also include
  // filtering, debounce, or more complex latching logic so the RIO doesn't
  // miss an input if it is reading at a slower rate.data


  // Writes can happen any time if you want to create a device that is more
  // interrupt driven.  Alternatively you can use a periodic send as shown
  // here.
  auto now = millis();
  if (now - lastSendMs > CAN_PERIOD_MS) {
      lastSendMs += CAN_PERIOD_MS;

      // zero memory buffer (this isn't strictly required, but did it for
      // debugging)
      memset(data, 0, CAN_PACKET_SIZE);

      // Distance sensor 0 MSB, LSB
      data[0] = distances[0] >> 8;
      data[1] = distances[0] & 0xFF;

      // Sensor 1 MSB, LSB
      data[2] = distances[1] >> 8;
      data[3] = distances[1] & 0xFF;

      // Sensor 2
      data[4] = distances[2] >> 8;
      data[5] = distances[2] & 0xFF;

      // Sensor 3
      data[6] = distances[3] >> 8;
      data[7] = distances[3] & 0xFF;


      // Send bytes. The API command is any 10 bit value specific to the device though
      // typically used for commands and configuration. Each packet is limited to up
      // to 8 bytes so you can use multiple API numbers to send larger packets. The
      // API number is arbitrary, but must match what is read on the RIO side.
      frcCANDevice.WritePacket(data, CAN_PACKET_SIZE, CAN_DEVICE_API);
  }

}

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

void drawClapped(void) {
  display.clearDisplay();

  display.drawBitmap(
    //(display.width()  - LOGO_WIDTH ) / 2,
    //(display.height() - LOGO_HEIGHT) / 2,
    0,
    0,
    clapped_warning_bmp, 128, 32, 1);
  display.display();
}

