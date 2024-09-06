
// UART Serial2(4, 5); // , 0, 0);

void setup() {
    delay(5000);
    Serial2.begin(57600);
    Serial.begin(115200);
    // ...
    Serial2.write("hello world");
    Serial.write("DONE world");
}

int incomingByte = 0; // for incoming serial data

void loop () {
    Serial.print("Bytes available for write: ");
    Serial.println(Serial2.availableForWrite());
    Serial2.write(12);
    if (Serial2.available() > 0) {
        // read the incoming byte:
        Serial.print("Total bytes: ");
        Serial.println(Serial2.available());
        incomingByte = Serial2.read();

        // say what you got:
        Serial.print("I received: ");
        Serial.println(incomingByte);
    } else {
        Serial.println("Nothing received...");
    }
    delay(5000);
}