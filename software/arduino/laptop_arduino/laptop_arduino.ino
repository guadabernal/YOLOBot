//motor controller

#define DIR_RIGHT 5
#define SPEED_RIGHT 6
#define INPUT_RIGHT 21

#define DIR_LEFT 8
#define SPEED_LEFT 9
#define INPUT_LEFT 20


void setup() {
  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(SPEED_LEFT, OUTPUT);
  pinMode(SPEED_RIGHT, OUTPUT);  

  pinMode(INPUT_LEFT, INPUT);
  pinMode(INPUT_RIGHT, INPUT);
  
  
  digitalWrite(DIR_LEFT, HIGH);
  digitalWrite(DIR_RIGHT, LOW);

  pinMode(INPUT_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_RIGHT), right_encoder, CHANGE);
  
  pinMode(INPUT_LEFT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INPUT_LEFT), left_encoder, CHANGE);
  
  Serial1.begin(57600);      //the connection to the computer via bluetooth
  Serial.begin(57600);     //serial is the bluetooth from the controller
  
}    
volatile int rticks = 0;
volatile int lticks = 0;

void right_encoder() {
  rticks++;  
}

void left_encoder() {
  lticks++;
}

long t0 = 0;
bool motoroff = true;

void loop() {
  long t1 = millis();
  int n = Serial1.available();
  
  if (n >= 4) {
    int16_t v[2] = {0};
    Serial1.readBytes((char*)v, 4);
    setThrottle(v[0], v[1]);
    t1 = millis();
    t0 = t1;
    motoroff=false;
  }
  if (t1 - t0 > 100 && !motoroff) {
    int16_t v[2] = {0}; //-128..127 
    n = Serial1.available();
    if (n > 0) Serial1.readBytes((char*)v, n);
      setThrottle(v[0], v[1]);
    motoroff = true;    
  }    
  delay(10);
}


void setThrottle(float TL, float TR) 
{
  float val=0;
  float val2=0;
  //val = analogRead(INPUT_LEFT); 
  //val2 = analogRead(INPUT_RIGHT); 
  //Serial.print("LeftSpeed=");
  //Serial.print(val);
  //Serial.print(" - RightSpeed=");
  //Serial.println(val2);
  
  if (TL >= 0) digitalWrite(DIR_LEFT, HIGH);
  else digitalWrite(DIR_LEFT, LOW);
 
  if (TR >= 0) digitalWrite(DIR_RIGHT, LOW);
  else digitalWrite(DIR_RIGHT, HIGH);
  
  analogWrite(SPEED_LEFT, abs(TL));
  analogWrite(SPEED_RIGHT, abs(TR));

  Serial.print("TL=");
  Serial.print(abs(TL));
  Serial.print(" - TR=");
  Serial.println(TR);
}

