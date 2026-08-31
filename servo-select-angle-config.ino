#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr int SDA_PIN = 22;
constexpr int SCL_PIN = 23;

constexpr uint8_t PCA_ADDRESS = 0x40;
constexpr uint8_t SERVO_COUNT = 10;

constexpr uint16_t SERVO_MIN_US = 600;
constexpr uint16_t SERVO_MAX_US = 2400;
constexpr uint16_t SERVO_FREQUENCY_HZ = 50;

constexpr int MOVE_STEP_DEGREES = 1;
constexpr int MOVE_STEP_DELAY_MS = 15;

Adafruit_PWMServoDriver pwm(PCA_ADDRESS, Wire);

int selectedServo = 0;
int commandedAngle[SERVO_COUNT];
bool waitingForServoNumber = false;

String inputLine;

uint16_t angleToTicks(int angle) {
  const uint16_t pulseUs = map(
    angle,
    0,
    180,
    SERVO_MIN_US,
    SERVO_MAX_US
  );

  // One 50 Hz period is 20,000 microseconds.
  return static_cast<uint32_t>(pulseUs) * 4096UL / 20000UL;
}

void writeServoAngle(uint8_t servo, int angle) {
  pwm.setPWM(servo, 0, angleToTicks(angle));
}

void moveServo(uint8_t servo, int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);

  // The first command moves directly because the previous position is unknown.
  if (commandedAngle[servo] < 0) {
    writeServoAngle(servo, targetAngle);
    commandedAngle[servo] = targetAngle;

    Serial.printf(
      "Servo %u commanded to %d degrees.\n",
      servo,
      targetAngle
    );

    return;
  }

  int currentAngle = commandedAngle[servo];

  while (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) {
      currentAngle += MOVE_STEP_DEGREES;

      if (currentAngle > targetAngle) {
        currentAngle = targetAngle;
      }
    } else {
      currentAngle -= MOVE_STEP_DEGREES;

      if (currentAngle < targetAngle) {
        currentAngle = targetAngle;
      }
    }

    writeServoAngle(servo, currentAngle);
    delay(MOVE_STEP_DELAY_MS);
  }

  commandedAngle[servo] = targetAngle;

  Serial.printf(
    "Servo %u commanded to %d degrees.\n",
    servo,
    targetAngle
  );
}

void disableServo(uint8_t servo) {
  pwm.setPin(servo, 0);
  commandedAngle[servo] = -1;

  Serial.printf("Servo %u disabled.\n", servo);
}

void disableAllServos() {
  for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
    pwm.setPin(servo, 0);
    commandedAngle[servo] = -1;
  }

  Serial.println("All servos disabled.");
}

void printPose() {
  Serial.println();
  Serial.println("CURRENT COMMANDED POSE");

  for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
    Serial.printf("Servo %u: ", servo);

    if (commandedAngle[servo] < 0) {
      Serial.println("not commanded");
    } else {
      Serial.printf("%d degrees\n", commandedAngle[servo]);
    }
  }

  Serial.println();
  Serial.print("Copyable array: {");

  for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
    if (commandedAngle[servo] < 0) {
      Serial.print("-1");
    } else {
      Serial.print(commandedAngle[servo]);
    }

    if (servo < SERVO_COUNT - 1) {
      Serial.print(", ");
    }
  }

  Serial.println("}");
}

void printHelp() {
  Serial.println();
  Serial.println("COMMANDS");
  Serial.println("s       Select a servo");
  Serial.println("0-9     Servo number after s");
  Serial.println("0-180   Move the selected servo");
  Serial.println("p       Print all commanded angles");
  Serial.println("x       Disable the selected servo");
  Serial.println("xa      Disable all servos");
  Serial.println("h       Print this help");
  Serial.println();
  Serial.printf("Selected servo: %d\n", selectedServo);
}

bool parseInteger(const String& text, int& result) {
  if (text.length() == 0) {
    return false;
  }

  char* endPointer = nullptr;
  const long value = strtol(text.c_str(), &endPointer, 10);

  if (*endPointer != '\0') {
    return false;
  }

  result = static_cast<int>(value);
  return true;
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  if (waitingForServoNumber) {
    int servoNumber;

    if (!parseInteger(command, servoNumber) ||
        servoNumber < 0 ||
        servoNumber >= SERVO_COUNT) {
      Serial.println("Enter a servo number from 0 to 9.");
      return;
    }

    selectedServo = servoNumber;
    waitingForServoNumber = false;

    Serial.printf("Selected servo %d.\n", selectedServo);
    Serial.println("Enter an angle from 0 to 180.");
    return;
  }

  if (command == "s") {
    waitingForServoNumber = true;
    Serial.println("Enter servo number 0 to 9.");
    return;
  }

  // Also accepts commands such as "s 4".
  if (command.startsWith("s ")) {
    String servoText = command.substring(2);
    servoText.trim();

    int servoNumber;

    if (!parseInteger(servoText, servoNumber) ||
        servoNumber < 0 ||
        servoNumber >= SERVO_COUNT) {
      Serial.println("Servo number must be from 0 to 9.");
      return;
    }

    selectedServo = servoNumber;
    Serial.printf("Selected servo %d.\n", selectedServo);
    return;
  }

  if (command == "p") {
    printPose();
    return;
  }

  if (command == "x") {
    disableServo(selectedServo);
    return;
  }

  if (command == "xa") {
    disableAllServos();
    return;
  }

  if (command == "h") {
    printHelp();
    return;
  }

  int angle;

  if (!parseInteger(command, angle) || angle < 0 || angle > 180) {
    Serial.println("Enter an angle from 0 to 180, or enter h.");
    return;
  }

  moveServo(selectedServo, angle);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (uint8_t servo = 0; servo < SERVO_COUNT; servo++) {
    commandedAngle[servo] = -1;
  }

  if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("ERROR: Could not start I2C.");
    while (true) {
      delay(1000);
    }
  }

  if (!pwm.begin()) {
    Serial.println("ERROR: PCA9685 not found.");
    while (true) {
      delay(1000);
    }
  }

  pwm.setPWMFreq(SERVO_FREQUENCY_HZ);
  delay(20);

  disableAllServos();

  Serial.println("Servo debugger ready.");
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    const char character = Serial.read();

    if (character == '\n' || character == '\r') {
      if (inputLine.length() > 0) {
        processCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += character;

      if (inputLine.length() > 32) {
        inputLine = "";
        Serial.println("Input too long.");
      }
    }
  }
}
