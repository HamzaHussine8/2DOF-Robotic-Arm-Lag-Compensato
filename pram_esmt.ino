float kp=1.000;    //change value of Kp
float ki=0.0000;   //change value of Ki
float kd=0.0;   // change value of Kd
unsigned long t_now;
unsigned long t_prev = 0;
const byte interruptPinA = 2;
const byte interruptPinB = 3;
volatile long EncoderCount = 0;
const byte DirPin1 = 5;
const byte DirPin2 = 6;//DirPin 1 and 2 will decide the motor rotation direction
//const byte PWMPin = 6; // This pin will control the Motor speed via PWMval
int PWMval=0; // Initial PWMval is set to zero, this value is the PWM duty cycle
volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta_now; 
float Theta_prev = 0;
float RPM_output, RPM_input;
int dt;                      // Period of time used to calcuate RPM
float RPM_max = 210;         // Setting up a safe maximum RPM for the Motor
#define pi 3.1416
float Vmax = 6;      // Check Motor Datasheet
float Vmin = 0;
float V = 0;         // set initial voltage to zero
float error_now, error_prev = 0, integ_now, integ_prev = 0;
void ISR_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);
  if (PinB == LOW) {
    if (PinA == HIGH) {
     EncoderCount++;
    }
   }

  else {
     if (PinA == LOW){
      EncoderCount++;
    }
  }
}
void ISR_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
      if (PinB == LOW) {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    }
}
void WriteDriverVoltage(float V, float Vmax) {
  int PWMval = int(255 * abs(V) / Vmax);
  if (PWMval > 255) {
    PWMval = 255;
  }
  //setting motor direction
  analogWrite(DirPin1, PWMval);
  analogWrite(DirPin2, 0);
}
//INTERRUPT SERVICE ROUTINE************************
ISR(TIMER1_COMPA_vect) {
  count++;
}
void setup() {
  //General setup

  Serial.begin(9600);
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);

//INTERRUPT SETUP***********************************
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}
//**************************************************************
//**************************************************************
//*MAIN LOOP************************************
void loop() {
  if (count > count_prev) {
    t_now = millis();
    Theta_now = EncoderCount / 834.0;
    dt = (t_now - t_prev);
    RPM_input = 200;       // This is the user input RPM
    if (t_now / 1000.0 > 100) {
      RPM_input = 0;
    }

    //ERROR AND PWM CALCULATIONS******************
    //PID MOTOR TERMS CALCULATIONS****************
    RPM_output = (Theta_now - Theta_prev) / (dt / 1000.0) * 60;
    error_now = RPM_input - RPM_output;
   integ_now = integ_prev + (dt * (error_now + error_prev) / 2);

   // Now calculating the Motor voltage or the PID controller ouput
    V = error_now*5;
    
    if (V > Vmax) {
      V = Vmax;
      integ_now = integ_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      integ_now = integ_prev;
    }
    WriteDriverVoltage(V, Vmax);
     Serial.print(RPM_input);
     Serial.print(" ");
     Serial.print(RPM_output);
     Serial.print(" ");
     Serial.println();
    Theta_prev = Theta_now;
    count_prev = count;
    t_prev = t_now;
    integ_prev = integ_now;
    error_prev = error_now;
}

}