#include <AccelStepper.h>
#include <Bounce2.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// Define pins for stepper motors
// Motor 1
#define M1_EN 20
#define M1_CW 19
#define M1_CLK 18

// Motor 2
#define M2_EN 23
#define M2_CW 22
#define M2_CLK 21

// Motor 3
#define M3_EN 40
#define M3_CW 38
#define M3_CLK 36

// Motor 4
#define M4_EN 34
#define M4_CW 32
#define M4_CLK 30

// Motor 5
#define M5_EN 29
#define M5_CW 27
#define M5_CLK 25

// Motor 6
#define M6_EN 28
#define M6_CW 26
#define M6_CLK 24

// Define pins for limit switches
#define LIMIT1_PIN 46
#define LIMIT2_PIN 48
#define LIMIT3_PIN 50
#define LIMIT4_PIN 52

// Define pin for servo motor
#define SERVO_PIN 44

// Define pins for LEDs (assign appropriate pins)
#define LED_RED_PIN    55
#define LED_YELLOW_PIN 56
#define LED_GREEN_PIN  57

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, M1_CLK, M1_CW);
AccelStepper stepper2(AccelStepper::DRIVER, M2_CLK, M2_CW);
AccelStepper stepper3(AccelStepper::DRIVER, M3_CLK, M3_CW);
AccelStepper stepper4(AccelStepper::DRIVER, M4_CLK, M4_CW);
AccelStepper stepper5(AccelStepper::DRIVER, M5_CLK, M5_CW);
AccelStepper stepper6(AccelStepper::DRIVER, M6_CLK, M6_CW);

// Create Bounce2 objects for limit switches
Bounce limitSwitch1 = Bounce();
Bounce limitSwitch2 = Bounce();
Bounce limitSwitch3 = Bounce();
Bounce limitSwitch4 = Bounce();

// Create Servo object
Servo servoMotor;

// Create MPU6050 object
MPU6050 mpu;

// Setup function
void setup() {
  Serial.begin(9600);

  // Initialize stepper motor enable pins
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT);
  pinMode(M3_EN, OUTPUT);
  pinMode(M4_EN, OUTPUT);
  pinMode(M5_EN, OUTPUT);
  pinMode(M6_EN, OUTPUT);

  // Enable all motors
  digitalWrite(M1_EN, LOW);
  digitalWrite(M2_EN, LOW);
  digitalWrite(M3_EN, LOW);
  digitalWrite(M4_EN, LOW);
  digitalWrite(M5_EN, LOW);
  digitalWrite(M6_EN, LOW);

  // Set max speed and acceleration for motors
  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);
  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);
  stepper5.setMaxSpeed(1000);
  stepper5.setAcceleration(500);
  stepper6.setMaxSpeed(1000);
  stepper6.setAcceleration(500);

  // Initialize limit switches with internal pull-up resistors
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(LIMIT3_PIN, INPUT_PULLUP);
  pinMode(LIMIT4_PIN, INPUT_PULLUP);

  // Attach Bounce2 to limit switches
  limitSwitch1.attach(LIMIT1_PIN);
  limitSwitch1.interval(5);
  limitSwitch2.attach(LIMIT2_PIN);
  limitSwitch2.interval(5);
  limitSwitch3.attach(LIMIT3_PIN);
  limitSwitch3.interval(5);
  limitSwitch4.attach(LIMIT4_PIN);
  limitSwitch4.interval(5);

  // Initialize servo motor
  servoMotor.attach(SERVO_PIN);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Initialize LEDs
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
}

// Main loop function
void loop() {
  // Update limit switches
  limitSwitch1.update();
  limitSwitch2.update();
  limitSwitch3.update();
  limitSwitch4.update();

  // Example: Zeroing Motor 1 using Limit Switch 1
  if (/* condition to zero Motor 1 */) {
    zeroMotor(stepper1, limitSwitch1);
  }

  // Example: Zeroing Motors 2 and 3 together using Limit Switch 2
  if (/* condition to zero Motors 2 and 3 */) {
    zeroDualMotors(stepper2, stepper3, limitSwitch2);
  }

  // Example: Zeroing Motor 4 using Limit Switch 3
  if (/* condition to zero Motor 4 */) {
    zeroMotor(stepper4, limitSwitch3);
  }

  // Example: Zeroing Motor 6 using Limit Switch 4
  if (/* condition to zero Motor 6 */) {
    zeroMotor(stepper6, limitSwitch4);
  }

  // Control Servo Motor (example)
  servoMotor.write(90); // Move to 90 degrees

  // Read MPU6050 data for Motor 5 positioning
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Process MPU6050 data as needed

  // Run steppers
  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  stepper5.run(); // Control based on MPU6050 data
  stepper6.run();

  // Control LEDs (example)
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  // Add delays or additional logic as needed
}

// Function to zero a single motor using a limit switch
void zeroMotor(AccelStepper &motor, Bounce &limitSwitch) {
  motor.moveTo(-100000); // Move in negative direction
  while (!limitSwitch.read()) {
    motor.run();
    limitSwitch.update();
  }
  motor.stop();
  motor.setCurrentPosition(0); // Set current position as zero
}

// Function to zero two motors simultaneously using a single limit switch
void zeroDualMotors(AccelStepper &motorA, AccelStepper &motorB, Bounce &limitSwitch) {
  motorA.moveTo(-100000);
  motorB.moveTo(-100000);
  while (!limitSwitch.read()) {
    motorA.run();
    motorB.run();
    limitSwitch.update();
  }
  motorA.stop();
  motorB.stop();
  motorA.setCurrentPosition(0);
  motorB.setCurrentPosition(0);
}
