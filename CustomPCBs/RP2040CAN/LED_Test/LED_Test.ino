const int greenLed = 0;
const int redLed = 1;

const int analogConnector = A2;   // GPIO 28, A2

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(analogConnector, OUTPUT);

}

void loop() {
  digitalWrite(redLed, HIGH);
  //digitalWrite(greenLed, LOW);
  digitalWrite(analogConnector, HIGH);
  delay(200);
  digitalWrite(redLed, LOW);
  //digitalWrite(greenLed, HIGH);
  digitalWrite(analogConnector, LOW);
  delay(200);
}
