#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X range_sensor = Adafruit_VL53L0X();

int xshut = 20;

void setup() {
  // Initialize sensor by toggling power
  

  pinMode(xshut, OUTPUT);

  digitalWrite(xshut, LOW);
  delay(200);
  digitalWrite(xshut, HIGH);
  
  

  // start serial moniter for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; //wait for serial port to connect. Needed for native USB port only
  }

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
}

void loop() {
  char buffer[100];
  int distance;
  int angle;


  // Check if the range sensor has compleated a range mesurement
  if (range_sensor.isRangeComplete()){

    // Get the intager range value in mm
    distance = range_sensor.readRange();

    /*
    Serial.print(0); // To freeze the lower limit
    Serial.print(" ");
    Serial.print(9000); // To freeze the upper limit
    Serial.print(" ");
    */

    // Debug serial print 
    sprintf (buffer, "Distance:%dmm\n",distance);
    Serial.print(buffer);

    // Divide the range by 8 (bit shift 2^3 times to the right)
    angle = distance >> 3;
  }
}
