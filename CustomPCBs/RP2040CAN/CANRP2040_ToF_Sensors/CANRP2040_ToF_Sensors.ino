/*
TEAM 4917 SIR LANCERBOT 2026

Download the Adafruit board package for running the RP2040 - it allows us to 
command the GPIO of the RP2040 directly while it thinks that it is an 
Arduin Nano RP2040 connect. (If it can dream it, it can be it)
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
Adafruit_VL53L0X range_sensor = Adafruit_VL53L0X();

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


// Define hardware locations
const int greenLed = 0;
const int redLed = 1;
const int adc0 = A0;
const int adc1 = A1;
const int adcX = A2;

int xshut0 = 18;
int xshut1 = 19;
int xshut2 = 16;
int xshut3 = 17;

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

void setup() {
  // Give the OLED time to boot
  delay(1000);

  // Intialize the SPI port
  mcp2515.test_init();

  // Begin serial
  Serial.begin(115200);




  // Initialize sensor by toggling power
  pinMode(xshut0, OUTPUT);
  pinMode(xshut1, OUTPUT);
  pinMode(xshut2, OUTPUT);
  pinMode(xshut3, OUTPUT);

  digitalWrite(xshut0, LOW);
  digitalWrite(xshut1, LOW);
  digitalWrite(xshut2, LOW);
  digitalWrite(xshut3, LOW);

  Serial.print("Starting sensor\n");
  
  // Initialize range sensor driver
  if (!range_sensor.begin()) {
    Serial.print("Failed to boot VL53L0X. Stopping.\n");
    while(1);

  } else {
    Serial.print("works??");
  }

  Serial.print("VL53L0X driver loaded.\n");
  range_sensor.startRangeContinuous();



  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  

  // Define other hardware
  pinMode(greenLed, OUTPUT);
  pinMode(redLed, OUTPUT);

  
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
  
  

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  drawLogo();    // Draw a bitmap image (logo)
  display.display();
  delay(2000);

  // Initialization Complete 
}

void loop() {
  // Display shenanigans 
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  //int sensor0 = analogRead(adc0);
  //int sensor1 = analogRead(adc1);
  int sensor0 = getSensorData(0);


  drawProgressBar(60, 0, sensor0);
  //drawProgressBar(60, 7, sensor1);

  char output[255];
  //sprintf(output, "ADC0: %d\nADC1: %d", sensor0, sensor1);
  sprintf(output, "ToF 1: %d\nToF 2:", sensor0);

  display.write(output);

  display.display();
  

  int16_t distance;
  int16_t ir0;
  int16_t ir1;
  uint8_t data[CAN_PACKET_SIZE];

  // Update must be called every loop in order to receive messages
  frc::CAN::Update();

  // Update all sensors as frequently as possible. This could also include
  // filtering, debounce, or more complex latching logic so the RIO doesn't
  // miss an input if it is reading at a slower rate.data

  /*
  // Update distance on every loop
  if (range_sensor.isRangeComplete()) {
    // Get the intager range value in mm
    distance = range_sensor.readRange();
  }
  */

  // Read the analog IR sensor
  ir0 = analogRead(A0);
  ir1 = analogRead(A1);

  // Writes can happen any time if you want to create a device that is more
  // interrupt driven.  Alternatively you can use a periodic send as shown
  // here.
  auto now = millis();
  if (now - lastSendMs > CAN_PERIOD_MS) {
      lastSendMs += CAN_PERIOD_MS;

      // zero memory buffer (this isn't strictly required, but did it for
      // debugging)
      memset(data, 0, CAN_PACKET_SIZE);

      // Distance sensor MSB, LSB
      data[0] = ir0 >> 8;
      data[1] = ir0 & 0xFF;

      // Analog 0 sensor MSB, LSB
      data[2] = ir1 >> 8;
      data[3] = ir1 & 0xFF;

      //data[4] = analog1 >> 8;
      //data[5] = analog1 & 0xFF;


      // Send bytes. The API command is any 10 bit value specific to the device though
      // typically used for commands and configuration. Each packet is limited to up
      // to 8 bytes so you can use multiple API numbers to send larger packets. The
      // API number is arbitrary, but must match what is read on the RIO side.
      frcCANDevice.WritePacket(data, CAN_PACKET_SIZE, CAN_DEVICE_API);
  }

}


int getSensorData(int sensor) {
  char buffer[100];
  int distance;
  int angle;


  // Check if the range sensor has compleated a range mesurement
  if (range_sensor.isRangeComplete()){

    // Get the intager range value in mm
    distance = range_sensor.readRange();

    
    Serial.print(0); // To freeze the lower limit
    Serial.print(" ");
    Serial.print(9000); // To freeze the upper limit
    Serial.print(" ");
    

    // Debug serial print 
    sprintf (buffer, "Distance:%dmm\n",distance);
    Serial.print(buffer);

    // Divide the range by 8 (bit shift 2^3 times to the right)
    angle = distance >> 3;

    return distance;
  
  } else {
    return -1;
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


int scaleSensorValue(int sensor) {
  // Need a value between 0 & 60
  // Sensor values are 60 & 900
  return (sensor/17);
}


void drawProgressBar(int xStart, int yStart, int sensor) {
  int barWidth = 60;
  int barHeight = 6;

  display.drawRect(xStart, yStart, barWidth, barHeight, SSD1306_WHITE);
  display.fillRect(xStart, yStart, scaleSensorValue(sensor), barHeight, SSD1306_WHITE);
}

