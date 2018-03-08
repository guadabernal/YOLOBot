#include <robot.h>

// Pin definitions
#define PIN_DIR_RIGHT 5
#define PIN_SPEED_RIGHT 6
#define PIN_INPUT_RIGHT 21

#define PIN_DIR_LEFT 8
#define PIN_SPEED_LEFT 9
#define PIN_INPUT_LEFT 20

Robot robot;

volatile int rTicks = 0;
volatile int lTicks = 0;

void right_encoder()
{
  rTicks++;
}

void left_encoder()
{
  lTicks++;
}

void resetEncoders()
{
  rTicks = 0;
  lTicks = 0;
}


// Initialization
void setup()
{
  // Pin mode definitions
  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_DIR_RIGHT, OUTPUT);
  pinMode(PIN_SPEED_LEFT, OUTPUT);
  pinMode(PIN_SPEED_RIGHT, OUTPUT);  
  pinMode(PIN_INPUT_RIGHT, INPUT_PULLUP);  
  pinMode(PIN_INPUT_LEFT, INPUT_PULLUP);
  // Encoder interruptions
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_RIGHT), right_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_INPUT_LEFT), left_encoder, CHANGE);
  // Initial direction and speed
  digitalWrite(PIN_DIR_LEFT, HIGH);
  digitalWrite(PIN_DIR_RIGHT, LOW);
  digitalWrite(PIN_SPEED_LEFT, LOW);
  digitalWrite(PIN_SPEED_RIGHT, LOW);

  // Serial communication initialization
  Serial.begin(57600);   // Debugging laptop usb port
  Serial1.begin(57600);  // Bluetooth from the controller/laptop
}    


void loop()
{
  int n = Serial1.available();
  
  if (n >= 4) {
    int16_t v[2] = {0};
    Serial1.readBytes((char*)v, 4);
    robot.executeCommand(v[0], v[1]);
  }

  robot.update(lTicks, rTicks);
  delay(10);
}




// long t0 = 0;
// bool motoroff = true;

// void loop() {
//   long t1 = millis();
//   int n = Serial1.available();
  
//   if (n >= 4) {
//     int16_t v[2] = {0};
//     Serial1.readBytes((char*)v, 4);
//     setThrottle(v[0], v[1]);
//     t1 = millis();
//     t0 = t1;
//     motoroff=false;
//   }
//   if (t1 - t0 > 100 && !motoroff) {
//     int16_t v[2] = {0}; //-128..127 
//     n = Serial1.available();
//     if (n > 0) Serial1.readBytes((char*)v, n);
//       setThrottle(v[0], v[1]);
//     motoroff = true;    
//   }    
//   delay(10);
// }


// void setThrottle(float TL, float TR) 
// {
//   float val=0;
//   float val2=0;
  
//   if (TL >= 0) digitalWrite(PIN_DIR_LEFT, HIGH);
//   else digitalWrite(PIN_DIR_LEFT, LOW);
 
//   if (TR >= 0) digitalWrite(PIN_DIR_RIGHT, LOW);
//   else digitalWrite(PIN_DIR_RIGHT, HIGH);
  
//   analogWrite(PIN_SPEED_LEFT, abs(TL));
//   analogWrite(PIN_SPEED_RIGHT, abs(TR));

//   Serial.print("TL=");
//   Serial.print(abs(TL));
//   Serial.print(" - TR=");
//   Serial.println(TR);
// }

