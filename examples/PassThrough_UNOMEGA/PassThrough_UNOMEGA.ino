
// Connect HM10 Arduino Uno
// Pin 1/TXD Pin 7
// Pin 2/RXD Pin 8

// Connect HM10 Arduino Mega
// Pin 1/TXD Pin 15
// Pin 2/RXD Pin 8

#if defined(ARDUINO_AVR_UNO)
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(7, 8); // RX, TX
#endif

#if defined(ARDUINO_AVR_MEGA2560)
  #define mySerial Serial3
#endif

#define DEVICE_SPEED 9600
#define CONSOLE_SPEED 9600

void setup() {
  Serial.begin(CONSOLE_SPEED);
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);
  mySerial.begin(DEVICE_SPEED);
  Serial.println("Ready");
}

void loop() {
  char c;
  if (Serial.available()) {
    c = Serial.read();
    mySerial.print(c);
  }
  if (mySerial.available()) {
    c = mySerial.read();
    Serial.print(c);
  }
}
