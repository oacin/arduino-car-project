#define adjust1 A0
#define adjust2 A1

//Right Motor
int dir_Right_A1 = 3;// Direction/Break command pin 3.
int dir_Right_B1 = 4;// Direction/Break command pin 4.
int Right_Wheel_Pin = 5;// PWM Output on pin 5 

//Left Motor 
int dir_Left_A2 = 7;// Direction/Break command pin 7.
int dir_Left_B2 = 8;// Direction/Break command pin 8.
int Left_Wheel_Pin = 9;// PWM Output on pin 9.

int brakePin = 13,//brake pinout
    brake = 0,//digital value 0 or 1
    throttle = 0,//throttle var
    sentido = 0,//ligar chave para selecionar frente/tras
    VInner = 0,//inner wheel velocity
    VOuter = 0;//outer wheel velocity

float B = 65,//distance between wheels vertically
      L = 128,//distance between wheels horizontally
      adc1 = 0,//storing var for potentiometer value
      adc2 = 0,//storing var for potentiometer value
      minAdc2 = 0,//potentiometer max value
      midpointAdc2 = 512,//potentiometer mid value
      maxAdc2 = 1023,//potentiometer max value
      minSteering = -40,
      maxSteering = 40,
      angle = 0,
      pi = 3.141592;

//FUNCTIONS
void forward() {
  digitalWrite (dir_Right_A1, HIGH);//FORWARD right wheel
  digitalWrite (dir_Right_B1, LOW);//FORWARD right wheel
  digitalWrite (dir_Left_A2, LOW);//FORWARD left wheel
  digitalWrite (dir_Left_B2, HIGH);//FORWARD left wheel 

  if(angle < -10) {//left
    analogWrite(Left_Wheel_Pin, VInner);
    analogWrite(Right_Wheel_Pin, VOuter);
  } else if(angle > 10) {//right
    analogWrite(Left_Wheel_Pin, VOuter);
    analogWrite(Right_Wheel_Pin, VInner);
  } else if(angle >= -10 && angle <= 10) {//forward
    analogWrite(Left_Wheel_Pin, throttle);
    analogWrite(Right_Wheel_Pin, throttle);
  }
}
  
  
void backward() {
  digitalWrite (dir_Right_A1, LOW);//FORWARD right wheel
  digitalWrite (dir_Right_B1, HIGH);//FORWARD right wheel
  digitalWrite (dir_Left_A2, HIGH);//FORWARD left wheel
  digitalWrite (dir_Left_B2, LOW);//FORWARD left wheel 

  if(angle < -10) {//left
    analogWrite(Left_Wheel_Pin, VInner);
    analogWrite(Right_Wheel_Pin, VOuter);
  } else if(angle > 10) {//right
    analogWrite(Left_Wheel_Pin, VOuter);
    analogWrite(Right_Wheel_Pin, VInner);
  } else if(angle >= -10 && angle <= 10) {//forward
    analogWrite(Left_Wheel_Pin, throttle);
    analogWrite(Right_Wheel_Pin, throttle);
  }
}


void parar() {
  digitalWrite (dir_Right_A1, LOW);//Break right wheel
  digitalWrite (dir_Right_B1, LOW);//Break right wheel
  digitalWrite (dir_Left_A2, LOW);//Break left wheel
  digitalWrite (dir_Left_B2, LOW);//Break left wheel 
}

void setup() {
  Serial.begin(9600);
  pinMode(adjust1, INPUT);
  pinMode(adjust2, INPUT);
  pinMode(brakePin, INPUT);
  pinMode(dir_Left_A2, OUTPUT);
  pinMode(dir_Left_B2, OUTPUT);
  pinMode(Left_Wheel_Pin, OUTPUT);
  pinMode(dir_Right_A1, OUTPUT);
  pinMode(dir_Right_B1, OUTPUT);
  pinMode(Right_Wheel_Pin, OUTPUT);
}

void loop() {
  while(brake != 1) {
    adc1 = analogRead(adjust1);
    adc2 = analogRead(adjust2);//pontentiometer for steering (0 <---> 1024)
    throttle = map(adc1, 0, 1023, 0, 254);//throttle
    
    angle = -maxSteering+((maxSteering-minSteering)*adc2/maxAdc2);//between (-40 <---> 40)
    VInner = throttle*(1-(B*tan(angle*(pi/180))/(2*L)));
    VOuter = throttle*(1+(B*tan(angle*(pi/180))/(2*L)));

    if(sentido == 0) {
      forward();
    } else if(sentido == 1){
      backward();
    }
    
    //brake = digitalRead(brakePin); reading case brake isnt pressed
    
    Serial.println(throttle);
    Serial.println(adc2);
    Serial.println(angle);
    Serial.println(VInner);
    Serial.println(VOuter);
    Serial.println("______");
  }

  parar();
  //brake = digitalRead(brakePin); reading case brake is pressed
  
  Serial.println(throttle);
  Serial.println(adc2);
  Serial.println(angle);
  Serial.println(VInner);
  Serial.println(VOuter);
  Serial.println("______");
}
