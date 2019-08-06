#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 130
#define SERVOMAX 570
#define trig_pin A1 //analog input 1
#define echo_pin A2 //analog input 2
#define Maxdist 200

NewPing sonar(trig_pin, echo_pin, Maxdist); //sensor function

Servo neck;
int lth = 12;
int lknee = 13;
int lankle = 14;
int lhip = 11;

int rth = 3;
int rknee = 2;
int rankle = 1;
int rhip = 4;

int lthcentre = 90;
int lkneecentre = 90;
int lhipcentre = 90;
int lanklecentre = 90;
int rthcentre = 90;
int rkneecentre = 90;
int rhipcentre = 100;
int ranklecentre = 100;
int pulselength;
int angle;
int center = 90;
int lastpos[16];
uint8_t servonum = 0;
byte _state;
char opp;
void offservo(void){
 pwm.setPWM(servonum, 0, 0); 
}
void balancestraight(void){
  move_servos(lth, lthcentre);
  move_servos(rth, rthcentre);
  move_servos(lhip, lhipcentre);
  move_servos(rhip, rhipcentre);
  move_servos(rknee, rkneecentre);
  move_servos(lknee, lkneecentre);
  move_servos(lankle, lanklecentre);
  move_servos(rankle, ranklecentre);
  }
void balanceleft(void) {
  move_servos(rankle, ranklecentre + 10);
  move_servos(lankle, lanklecentre + 10);
}


void balanceright(void) {
  move_servos(lankle, lanklecentre - 10);
  move_servos(rankle, ranklecentre - 10);
}
void centerankle(void) {
  move_servos(lankle, lanklecentre);
  move_servos(rankle, ranklecentre);
}
void head(){
  for(int i=90;i<=170;i++){
    neck.write(i);
  }
  delay(1000);
  for(int i=170;i>=10;i--){
    neck.write(i);
  }
  delay(1000);
  for(int i=10;i<=90;i++){
    neck.write(i);
  }
}
void setup() {
  neck.attach(9);
  neck.write(90);
  pwm.begin();
  Serial.begin(9600);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);
 balancestraight();
  delay(2000);

}
void loop() {
 if(Serial.available()){
  opp = Serial.read();
  switch (opp)
  {
    case 'f':
    walkforwad();
    break;
    case 'r':
    turnright();
    break;
    case 'l':
    turnleft();
    break();
    default:
    balancestraight();
    break;
   }
  }
}
void walk(){
  first_step_forward();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
  forward_right();
  delay(100);
  forward_left();
  delay(100);
}
void move_servos(byte servonum, short angle){
  int pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulselength);
  lastpos[servonum] = angle;
}
void move_servos(byte servonum1, byte servonum2, short angle1, short angle2){
  int pulselength1 = map(angle1, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum1, 0, pulselength1);
  lastpos[servonum1] = angle1;
  int pulselength2 = map(angle2, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum2, 0, pulselength2);
  lastpos[servonum2] = angle2;
}

/*
   function to do a first step of walking forward.
 */
void first_step_forward()
{
    move_servos(rankle, lankle, lastpos[rankle]+20, lastpos[lankle]+20);
    delay(10);
    move_servos(rhip, lhip, lastpos[rhip]-20, lastpos[lhip]-20);
    delay(10);
    move_servos(rhip, lastpos[rhip]-10);
    delay(10);
    move_servos(rankle, lastpos[rankle]-5);
    delay(10);
    move_servos(lknee, lth, lastpos[lknee]+30, lastpos[lth]+30);
    delay(10);
    move_servos(rhip, lastpos[rhip]+10);
    delay(10);
    move_servos(rankle, lastpos[rankle]+5);
    delay(10);
    move_servos(rankle, lankle, lastpos[rankle]-20, lastpos[lankle]-20);
    delay(10);
    move_servos(rhip, lhip,lastpos[rhip]+20, lastpos[lhip]+20);
    delay(10);
    _state = 1;
}


void forward_right()
{
    move_servos(rankle, lankle, lastpos[rankle]-10, lastpos[lankle]-10);
    delay(10);
    move_servos(rhip, lhip, lastpos[rhip]+10, lastpos[lhip]+10);
    delay(10);
    move_servos(lhip, lastpos[lhip]+10);
    delay(10);
    move_servos(rknee, rth, lastpos[rknee]-30, lastpos[rth]-30);
    delay(10);
    move_servos(lhip, lastpos[lhip]+5);
    delay(10);
    move_servos(lknee, lth, lastpos[lknee]-30, lastpos[lth]-30);
    delay(10);
    move_servos(rankle, lankle, lastpos[rankle]+10, lastpos[lankle]+10);
    delay(10);
    move_servos(rhip, lhip, lastpos[rhip]-10, lastpos[lhip]-10);
    delay(10);
    move_servos(lhip, lastpos[lhip]-15);
    delay(10);
    _state = 2;
}


void forward_left()
{
    move_servos(rankle, lankle, lastpos[rankle]+20, lastpos[lankle]+20);
    delay(10);
    move_servos(rhip, lhip, lastpos[rhip]-20, lastpos[lhip]-20);
    delay(10);
    move_servos(rhip, lastpos[rhip]-5);
    delay(10);
    move_servos(lknee, lth, lastpos[lknee]+30, lastpos[lth]+30);
    delay(10);
    move_servos(rhip, lastpos[rhip]-5);
    delay(10);
    move_servos(rknee, rth, lastpos[rknee] + 30, lastpos[rth] + 30);
    delay(10);
    move_servos(rhip, lastpos[rhip]+10);
    delay(10);
    move_servos(rankle, lankle, lastpos[rankle]-20, lastpos[lankle]-20);
    delay(10);
    move_servos(rhip, lhip, lastpos[rhip]+20, lastpos[lhip]+20);
    delay(10);
    _state = 3;
}

/** 
   function to walk backward by doing a right step.
 */
void backward_right()
{
  move_servos(s_right_1_, s_left_1_, -10, _delay); //delay(20);
  move_servos(s_right_4_, s_left_4_, 10, _delay);  //delay(10);
  move_servos(s_left_4_, 10, _delay);              //delay(10);
  move_servos(s_right_2_, s_right_3_, 30, _delay); //delay(10);
  move_servos(s_left_4_, 5, _delay);               //delay(10);
  move_servos(s_left_2_, s_left_3_, 30, _delay);   //delay(10);
  move_servos(s_right_1_, s_left_1_, 10, _delay);  //delay(20);
  move_servos(s_right_4_, s_left_4_, -10, _delay); //delay(10);
  move_servos(s_left_4_, -15, _delay);             //delay(10);
  _state = 3;
}

/** 
   function to walk backward by doing a left step.
 */
void backward_left()
{
  move_servos(s_right_1_, s_left_1_, 20, _delay);   //delay(20);
  move_servos(s_right_4_, s_left_4_, -20, _delay);  //delay(10);
  move_servos(s_right_4_, -10, _delay);             //delay(10);
  move_servos(s_left_2_, s_left_3_, -30, _delay);   //delay(10);
  move_servos(s_right_2_, s_right_3_, -30, _delay); //delay(10);
  move_servos(s_right_4_, 10, _delay);              //delay(10);
  move_servos(s_right_4_, s_left_4_, 20, _delay);   //delay(10);
  move_servos(s_right_1_, s_left_1_, -20, _delay);  //delay(20);
  _state = 2;
}

/** 
   function to return to idle from walking state.
 */
void idle_from_right()
{
  if (_state == 2)
  {
    move_servos(s_right_1_, s_left_1_, 10, _delay);
    delay(10);
    move_servos(s_right_2_, s_right_3_, 40, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, -10, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
    delay(10);

    _state = 0;
  }
}

/** 
   function to return to idle from walking state.
 */
void idle_from_left()
{
  if (_state == 3 || _state == 1)
  {
    move_servos(s_right_1_, s_left_1_, -10, _delay);
    delay(10);
    move_servos(s_left_2_, s_left_3_, -40, _delay);
    delay(10);
    move_servos(s_left_2_, 5, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
    delay(10);
    _state = 0;
  }
}

/** 
   function to lift front the right leg.
 */
void lift_front_right()
{
  idle_from_right();
  idle_from_left();

  if (_state == 0)
  {
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, 20, _delay);
    delay(10);
    move_servos(s_left_4_, 5, _delay);
    delay(10);
    move_servos(s_left_2_, -30, _delay);
    delay(10);
    move_servos(s_right_2_, -90, _delay);
    delay(50);
    move_servos(s_right_2_, -60, _delay);
    delay(50);
    _state = 4;
  }
}

/** 
   function to return to idle from frontal lifting of the right leg.
 */
void down_front_right()
{
  if (_state == 4)
  {
    move_servos(s_right_2_, -60, _delay);
    delay(50);
    move_servos(s_right_2_, -90, _delay);
    delay(50);
    move_servos(s_left_2_, -30, _delay);
    delay(10);
    move_servos(s_left_4_, 5, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, 20, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
    delay(10);
    _state = 0;
  }
}

/** 
   function to lift front the left leg.
 */
void lift_front_left()
{
  idle_from_right();
  idle_from_left();

  if (_state == 0)
  {
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
    delay(10);
    move_servos(s_right_4_, s_left_4_, -10, _delay);
    delay(10);
    move_servos(s_right_2_, 10, _delay);
    delay(10);
    move_servos(s_right_3_, -10, _delay);
    delay(10);
    move_servos(s_left_2_, 90, _delay);
    move_servos(s_left_2_, 60, _delay);
    _state = 6;
  }
}

/** 
   function to return to idle from frontal lifting of the left leg.
 */
void down_front_left()
{
  if (_state == 6)
  {
    move_servos(s_left_2_, -90, 7);
    delay(10);
    move_servos(s_left_2_, -60, 7);
    delay(10);
    move_servos(s_right_3_, 10, _delay);
    delay(10);
    move_servos(s_right_2_, -10, 7);
    delay(10);
    move_servos(s_right_4_, s_left_4_, 10, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
    _state = 0;
  }
}

/** 
   function to lift back the right leg.
 */
void lift_back_right()
{
  idle_from_right();
  idle_from_left();

  if (_state == 0)
  {
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, 20, _delay);
    delay(10);
    move_servos(s_left_2_, 120, _delay);
    delay(10);
    move_servos(s_right_2_, 10, _delay);
    _state = 5;
  }
}

/** 
   function to return to idle from back lifting of the right leg.
 */
void down_back_right()
{
  if (_state == 5)
  {
    move_servos(s_right_2_, -10, _delay);
    delay(10);
    move_servos(s_left_2_, -120, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, -20, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
    _state = 0;
  }
}

/** 
   function to lift back the left leg.
 */
void lift_back_left()
{
  idle_from_right();
  idle_from_left();

  if (_state == 0)
  {
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
    delay(10);
    move_servos(s_right_4_, s_left_4_, -10, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, -20, _delay);
    delay(10);
    move_servos(s_right_2_, -120, _delay);
    delay(10);
    move_servos(s_left_2_, -10, _delay);
    _state = 7;
  }
}

/** 
   function to return to idle from back lifting of the left leg.
 */
void down_back_left()
{
  if (_state == 7)
  {
    move_servos(s_left_2_, 10, _delay);
    delay(10);
    move_servos(s_right_2_, 120, _delay);
    delay(10);
    move_servos(s_right_1_, s_left_1_, 20, _delay);
    delay(10);
    move_servos(s_right_4_, s_left_4_, 10, _delay);
    delay(10);
    move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
    _state = 0;
  }
}

/** 
   function to shoot something with the right leg from idle state.
 */
void shoot_right()
{
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -20, _delay);
  delay(10);
  move_servos(s_right_1_, s_left_1_, 20, _delay);
  delay(10);
  move_servos(s_right_2_, 10, 3);
  delay(500);
  move_servos(s_right_2_, -60, 1);
  delay(500);
  move_servos(s_right_2_, 50, 3);
  move_servos(s_right_1_, s_left_1_, -20, _delay);
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
}

/** 
   function to shoot something with the left leg from idle state.
 */
void shoot_left()
{
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 25, _delay);
  delay(10);
  move_servos(s_right_4_, s_left_4_, -5, _delay);
  move_servos(s_right_1_, s_left_1_, -20, _delay);
  delay(10);
  move_servos(s_left_2_, -10, 3);
  delay(500);
  move_servos(s_left_2_, 60, 2);
  delay(500);
  move_servos(s_left_2_, -50, 3);
  move_servos(s_right_1_, s_left_1_, 20, _delay);
  move_servos(s_right_4_, s_left_4_, 5, _delay);
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -25, _delay);
}

/** 
   function to shoot something with the right leg from the walking state.
 */
void shoot_right_walk()
{
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -15, _delay);
  delay(10);
  move_servos(s_right_1_, s_left_1_, -15, _delay);
  delay(10);
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -10, _delay);
  delay(10);
  move_servos(s_right_4_, s_left_4_, 10, _delay);
  delay(10);
  move_servos(s_right_4_, 20, _delay);
  delay(10);
  move_servos(s_left_2_, s_left_3_, -40, 1);
  delay(10);
  move_servos(s_right_1_, s_left_1_, 20, _delay);
  delay(10);
  move_servos(s_right_2_, 10, 3);
  delay(500);
  move_servos(s_right_2_, -60, 1);
  delay(500);
  move_servos(s_right_2_, 50, 3);
  move_servos(s_right_1_, s_left_1_, -10, _delay);
  move_servos(s_right_4_, s_left_4_, -20, _delay);

  _state = 0;
}

/** 
   function to shoot something with the left leg from the walking state.
 */
void shoot_left_walk()
{
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 20, _delay);
  delay(10);
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, 30, _delay);
  delay(10);
  move_servos(s_right_4_, s_left_4_, -5, _delay);
  move_servos(s_right_1_, s_left_1_, -20, _delay);
  delay(10);
  move_servos(s_right_2_, s_right_3_, 30, 1);
  delay(10);
  move_servos(s_left_2_, -10, 3);
  delay(500);
  move_servos(s_left_2_, 60, 1);
  delay(500);
  move_servos(s_left_2_, -50, 3);
  move_servos(s_right_1_, s_left_1_, 20, _delay);
  move_servos(s_right_4_, s_left_4_, 5, _delay);
  move_servos_reverse(s_right_1_, s_left_1_, s_right_4_, s_left_4_, -30, _delay);

  _state = 0;
}

/** 
   function to split the robot.
 */
void split()
{
  if (_state == 0)
  {
    move_servos_reverse(s_left_1_, s_right_4_, s_right_1_, s_left_4_, 70, _delay);
    _state = 8;
  }
  else if (_state == 8)
  {
    move_servos_reverse(s_left_1_, s_right_4_, s_right_1_, s_left_4_, -70, _delay);
    _state = 0;
  }
}

/** 
   function to bend the robot.
 */
void bend()
{
  if (_state == 0)
  {
    move_servos_reverse(s_left_2_, s_left_5_, s_right_2_, s_left_5_, 100, _delay);
    _state = 9;
  }
  else if (_state == 9)
  {
    move_servos_reverse(s_left_2_, s_left_5_, s_right_2_, s_left_5_, -100, _delay);
    _state = 0;
  }
}
