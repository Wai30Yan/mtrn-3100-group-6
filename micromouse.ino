
// right motor PWM = 9, DIR = 10
// left motor PWM = 11, DIR = 12
int rightPWM = 9;
int rightDIR = 10;
int leftPWM = 11;
int leftDIR = 12;

void setup() {
    pinMode(11, OUTPUT);
    pinMode(9, OUTPUT);
}

void loop() {
    digitalWrite(rightDIR, HIGH);
    analogWrite(rightPWM, 255);
    digitalWrite(leftDIR, LOW);
    analogWrite(leftPWM, 0);

    delay(3000);

    digitalWrite(leftDIR, HIGH);
    analogWrite(leftPWM, 255);
    digitalWrite(rightDIR, LOW);
    analogWrite(rightPWM, 0);

    delay(3000);
}