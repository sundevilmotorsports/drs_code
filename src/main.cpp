#include <Servo.h>
#include <Arduino_CAN.h>
// Define servo objects
Servo servoDriverLeft;
Servo servoDriverRight;
//Constants
#define BUTTON_PIN 7
#define DEBOUNCE_DELAY 50//in ms
#define ACTUATION_DISTANCE 16//Servo Degrees
#define SERVO_DRIVER_RIGHT_PIN 11
#define SERVO_DRIVER_LEFT_PIN 9
#define MIN_PWM_TIME_MS 1000
//Globals
bool buttonState = LOW;
bool lastButtonState = LOW;
int initialservoPosDriverLeft = 0;
int initialservoPosDriverRight = 0;
unsigned long lastDebounceTimeMS = 0;

static uint32_t const CAN_ID = 0x367;
static uint32_t msg_cnt = 0;
int readVal = 0;


void canWrite(uint8_t msg_data[]) {
  memcpy((void *)(msg_data + 4), &msg_cnt, sizeof(msg_cnt));
  CanMsg const msg(CanStandardId(CAN_ID), sizeof(msg_data), msg_data);
  /* Transmit the CAN message, capture and display an
   * error core in case of failure.
   */
  if (int const rc = CAN.write(msg); rc < 0)
  {
    Serial.print  ("CAN.write(...) failed with error code ");
    Serial.println(rc);
  }
}

void setup() {
  pinMode(D4, INPUT);
  // Attach servo objects to corresponding pins
  //driver left
  servoDriverLeft.attach(SERVO_DRIVER_LEFT_PIN);
  servoDriverLeft.writeMicroseconds(MIN_PWM_TIME_MS);
  //driver right
  servoDriverRight.attach(SERVO_DRIVER_RIGHT_PIN);
  servoDriverRight.writeMicroseconds(MIN_PWM_TIME_MS);
 
  // Initialize button pin as input and enable pull down resistor
  pinMode(BUTTON_PIN, INPUT);
  // Save initial positions of servos

  initialservoPosDriverLeft = 150;
  initialservoPosDriverRight = 110;
  //Run Servo Homing Sequence  
  //Move rewards full distance
  /*servoDriverLeft.write(initialservoPosDriverRight + ACTUATION_DISTANCE);
  servoDriverRight.write(initialservoPosDriverLeft - ACTUATION_DISTANCE);
  delay(200);
  //Move to initial position
  servoDriverRight.write(initialservoPosDriverRight);
  servoDriverLeft.write(initialservoPosDriverLeft);
  */
 while (!Serial) { }
}

void loop() {

  //Run initial start up sequence
  int buttonInput;
  uint8_t msg_data[] = {};

  // Read button state and debounce
  buttonInput = digitalRead(BUTTON_PIN);
  buttonState = buttonInput;
  delay(10);//debounce 10ms
  //if pressed only actuate if it currently is pressed and the last state was LOW
  if (buttonState == HIGH && lastButtonState == LOW) {
    Serial.println(servoDriverLeft.read());
    Serial.println(servoDriverRight.read());
    // Set servo positions
    servoDriverLeft.write(initialservoPosDriverLeft+ACTUATION_DISTANCE);
    servoDriverRight.write(initialservoPosDriverRight-ACTUATION_DISTANCE);
    Serial.println("Pressed");
    Serial.println("------");
    Serial.println(servoDriverLeft.read());
    Serial.println(servoDriverRight.read());
    Serial.println();
    msg_data[0] = {1};
    canWrite(msg_data);
    }
    //if held just wait a sec
    else if (buttonState == HIGH && lastButtonState == HIGH){
      delay(5);
    }
    //button is released
    else {
        //Return servos to initial position
        servoDriverLeft.write(initialservoPosDriverLeft);
        servoDriverRight.write(initialservoPosDriverRight);
        Serial.println("Unpressed");
        msg_data[0] = {0};
        canWrite(msg_data);
      }
    lastButtonState = buttonInput;
    delay(50);//delay
  }


