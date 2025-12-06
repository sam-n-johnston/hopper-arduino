
// const int PULLER_PWM1 = 0, PULLER_PWM2 = 2, PULLER_DIAG = 4,
//           PULLER_EN = 6;
// const int PULLER_OCM = 27;
const int PULLER_PWM1 = 1, PULLER_PWM2 = 3, PULLER_DIAG = 5,
          PULLER_EN = 7;


// torque between 0 and 255
void setMotorTorque(int torque, bool direction) {
    digitalWrite(PULLER_EN, HIGH);
    if (direction) {
        digitalWrite(PULLER_PWM1, 0);
        analogWrite(PULLER_PWM2, torque);
    } else {
        analogWrite(PULLER_PWM1, torque);
        digitalWrite(PULLER_PWM2, 0);
    }
}

void setup()
{
    delay(5000);
    Serial.begin(115200);
    Serial.write("DONE world\n");

    // Disable Motor
    pinMode(PULLER_EN, OUTPUT);
    digitalWrite(PULLER_EN, LOW);

    pinMode(PULLER_PWM1, OUTPUT);
    pinMode(PULLER_PWM2, OUTPUT);
    // pinMode(PULLER_OCM, INPUT);
    // pinMode(PULLER_DIAG, INPUT);
    digitalWrite(PULLER_PWM1, LOW);
    digitalWrite(PULLER_PWM2, LOW);

    Serial.println("Setup complete\n");
}

void loop()
{
    setMotorTorque(50, true);
}
