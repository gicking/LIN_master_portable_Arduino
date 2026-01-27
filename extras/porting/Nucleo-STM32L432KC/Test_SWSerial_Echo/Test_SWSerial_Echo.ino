/*********************************

  Check if SoftwareSerial reading back 1-wire echo works as expected

  Setup:
    - connect pins D11=Tx with D10=Rx

  Procedure:
    - periodically send byte via SoftwareSerial, check if echo is received via Rx

*********************************/


#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(D10, D11); // Rx, Tx


void setup() {
    
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial);

    SoftSerial.begin(19200);
}

void loop() { // run over and over
    
    digitalWrite(LED_BUILTIN, HIGH);
    SoftSerial.write(0x55);
    digitalWrite(LED_BUILTIN, LOW);

    while (!SoftSerial.available());
    digitalWrite(LED_BUILTIN, HIGH);
    char c = SoftSerial.read();
    digitalWrite(LED_BUILTIN, LOW);

    Serial.print("received 0x");
    Serial.println((int) c, HEX);

    delay(300);

}
