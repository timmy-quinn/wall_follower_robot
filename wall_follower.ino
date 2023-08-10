/* 

 * Requirements: 
 * Robot avoids obstacles
 * Robot identifies soemthign next to it and attempts to follow alongside it

 * 
 */
  #include <Servo.h>
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 3   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4
#define LPT 2 // scan loop coumter

enum direction
{
  NONE=0,
  RIGHT,
  LEFT
};
direction wallDirection = NONE;

struct robot
{
  bool turn_left;
  bool turn_right;
  bool hard_turn; 
};

robot robot;

#define SERVO_PIN     9  //servo connect to D9

#define Echo_PIN    2   // Ultrasonic Echo pin connect to D11
#define Trig_PIN    10  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     13
#define FAST_SPEED  250     //both sides of the motor speed
#define SPEED  125     //both sides of the motor speed
#define SLOW_SPEED 30
#define TURN_SPEED  200     //both sides of the motor speed
#define BACK_SPEED1  255     //back speed
#define BACK_SPEED2  90     //back speed
#define bRIGHT 0b00001
#define bRIGHT_D 0b00010
#define bFORWARD 0b00100
#define bLEFT_D 0b01000
#define bLEFT 0b10000

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
const int wallLimit = 35;
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends moving (miliseconds)

int thereis;
Servo head;

void fitInt(int *integer, int fitValue )
{
  if(*integer>fitValue)
  {
    *integer = fitValue;
  }
  else if (*integer<(-1*fitValue))
  {
    *integer = fitValue*-1;
  }
}
/*motor control*/
void set_Motorspeeds(int speed_L,int speed_R)
{
  //If the right speed is positive, make motor go forward, else go backwards
  digitalWrite(RightDirectPin1, (speed_R > 0));
  digitalWrite(RightDirectPin2, (speed_R < 0));

  //If the left speed is positive, make motor go forward, else go backwards
  digitalWrite(LeftDirectPin1, (speed_L > 0));
  digitalWrite(LeftDirectPin2, (speed_L < 0));

  //Set the speed of each motor
  analogWrite(speedPinL,abs(speed_L)); 
  analogWrite(speedPinR,abs(speed_R));
}

void go_Forward(int speed)
{
  set_Motorspeeds(speed, speed);
}

void spin(int direction, int speed)
{
  if(direction==LEFT) set_Motorspeeds(-1*speed, speed);

  else if (direction==RIGHT) set_Motorspeeds(speed, -1*speed);
  
  else set_Motorspeeds(speed, speed);
}

void corner(int direction)
{
  if (direction == LEFT) set_Motorspeeds(SLOW_SPEED, 200);
  else if (direction == RIGHT) set_Motorspeeds(200, SLOW_SPEED);
}

void cornerBack(int direction)
{
  if (direction == LEFT) set_Motorspeeds(-30, -200);
  else if (direction == RIGHT) set_Motorspeeds(-200, -30);
}
void turn(int direction)
{
  if(direction == LEFT) set_Motorspeeds(SPEED, FAST_SPEED);
  else if (direction == RIGHT) set_Motorspeeds(FAST_SPEED, SPEED);
 
}

void reverse(int direction)
{
  if( direction == RIGHT ) set_Motorspeeds(SPEED, -1*FAST_SPEED);
  if(direction == LEFT ) set_Motorspeeds(-1*FAST_SPEED, SPEED);
  if(direction == NONE) set_Motorspeeds(-1*SPEED, -1*SPEED);
}

void stop()
{
  set_Motorspeeds(0,0);
}

void buzz_ON()   //open buzzer
{
  
  for(int i=0;i<100;i++)
  {
   digitalWrite(BUZZ_PIN,LOW);
   delay(2);//wait for 1ms
   digitalWrite(BUZZ_PIN,HIGH);
   delay(2);//wait for 1ms
  }
}

void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
  
}

void alarm(){
   buzz_ON();
 
   buzz_OFF();
}

/*detection of ultrasonic distance: total delay of 20 microseconds*/
int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}

//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
void watchSurroundings(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
  robot.turn_right = false;
  robot.turn_left = false;
  robot.hard_turn = false;
  centerscanval = watch();
  if(centerscanval < distancelimit)
  {
    robot.hard_turn = true;
    Serial.print("Centerscanval: ");
    Serial.println(centerscanval);
  }
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
     robot.turn_right = true;
     robot.hard_turn = true;
     if(wallDirection == NONE)
    {
      wallDirection = LEFT;
    }
    Serial.print("ldiagonalscanval: ");
    Serial.println(ldiagonalscanval);
  }
  head.write(180); 
  delay(300);
  leftscanval = watch();
  if(leftscanval < sidedistancelimit)
  {
    robot.turn_right = true;
    if(wallDirection == NONE)
    {
      wallDirection = LEFT;
    }
    Serial.print("leftscanval: ");
    Serial.println(leftscanval);
  }

  head.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit)
  {
    robot.hard_turn = true;
  }
  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  Serial.print("rdiagonalscanval: ");
  Serial.println(rdiagonalscanval);
  if(rdiagonalscanval<distancelimit)
  {
    robot.turn_left = true;
    robot.hard_turn = true; 
    if(wallDirection == NONE)
    {
      wallDirection = RIGHT;
    } 
  }
  head.write(0);
  delay(100);
  rightscanval = watch();
  Serial.print("rightscanval: ");
  Serial.println(rightscanval);
  if(rightscanval<sidedistancelimit)
  {
    robot.turn_left = true;
    if(wallDirection == NONE)
    {
      wallDirection = RIGHT;
    }

  }
  if(wallDirection==RIGHT) head.write(0);
  else if (wallDirection == LEFT) head.write(180);
  else head.write(90);
  Serial.println("*******************************");
  if(wallDirection == RIGHT) Serial.println("Wall direction Right");
  else if (wallDirection == LEFT) Serial.println("Wall direction Left");
  else if (wallDirection == NONE) Serial.println("Wall NOT FOUND");
  Serial.print("Turn right = ");
  Serial.println(robot.turn_right);
  Serial.print("Turn left = ");
  Serial.println(robot.turn_left);
  Serial.print("Hard turn = ");
  Serial.println(robot.hard_turn);
  Serial.println("");
  delay(100);
}

void contWallFollowing(int time)
{
  int loops = 0;
  int leftSpeed = SPEED;
  int rightSpeed = SPEED;
  int offset;
  int distance;
  while(loops<(time/10))
    {
      // stop();
      distance = watch();
      offset = ( - wallLimit)*3;
      fitInt(&offset, 125);
      Serial.println(offset);
      Serial.print("Walldirection");
      Serial.println(wallDirection);
      if (wallDirection == RIGHT)
      {
        set_Motorspeeds((125 + offset), (125 - offset));
      }
      else if(wallDirection == LEFT)
      {
        set_Motorspeeds((125 - offset), (125 + offset));

      }
      delay(10);
    }

}

//Maintain distance from the identified wall
void followWall()
{
  if (wallDirection == NONE)
  {
    //* go forward
    go_Forward(SPEED);
    delay(backtime);
    stop();
  }
  // else if (wallDirection == LEFT && leftscanval >= (wallLimit*1.3))
  // {
  //   //turn left
  //   corner(LEFT);
  //   delay(turntime);
  //   stop();
  // }
  else if (wallDirection == LEFT && leftscanval >= wallLimit)
  {
    //turn left
    turn(LEFT);
    delay(turntime);
    stop();
  }
  else if (wallDirection == LEFT && leftscanval < wallLimit)
  {
    //turn right
    turn(RIGHT); 
    delay(turntime);
    stop();

  }
  // else if (wallDirection == RIGHT && rightscanval >= (wallLimit*1.3))
  // {
  //   //turn right
  //   corner(RIGHT);
  //   delay(turntime);
  //   stop();
  // }
  else if (wallDirection == RIGHT && rightscanval > (wallLimit))
  {
    //turn right
    turn(RIGHT);
    delay(turntime);
    stop();
  }
  else if (wallDirection == RIGHT && rightscanval < wallLimit)
  {
    //turn left
    turn(LEFT);
    delay(turntime);
  }
  else
  {
    go_Forward(SPEED);
  }
  stop();
}

//navigate through a room, finding and following a wall
void auto_navigation(){

    //Adjust Course
    watchSurroundings();
    // if (robot.turn_left && robot.turn_right && robot.hard_turn && (leftscanval < distancelimit || ldiagonalscanval < distancelimit))
    // {
    //   //backup right
    //   reverse(RIGHT);
    //   delay(backtime);
    //   stop();
    // }
    // else if (robot.turn_left && robot.turn_right && robot.hard_turn)
    // {
    //   //backup left;
    //   reverse(LEFT);
    //   delay(backtime);
    //   stop();
    // }
    if (robot.turn_right && robot.hard_turn)
    {
      //spin right
      spin(RIGHT, TURN_SPEED);
      delay(turntime);
      stop();
    }
    else if (robot.hard_turn)
    {
      // spin left
      spin(LEFT, TURN_SPEED);
      delay(turntime);
      stop();
    }
    else if (robot.turn_left)
    {
      // turn left
      turn(LEFT);
      delay(turntime);
      stop();
      
    }
    else if (robot.turn_right)
    {
      //turn right
      turn(RIGHT);
      delay(turntime);
      stop();
    }
    else
    {
      //follow wall
      followWall();
    } 
    stop();
}

void setup() {
  /*setup L298N pin mode*/
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop();//stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT); 
  pinMode(Echo_PIN,INPUT); 
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);  
  buzz_OFF(); 

  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  head.attach(SERVO_PIN); 
  head.write(90);
   delay(2000);
  
  Serial.begin(9600);
 
 
}
void findPark(int time)
{
  int loops = 0;
  int leftSpeed = SPEED;
  int rightSpeed = SPEED;
  int offset;
  int distance;
  int distance, prevDistance;
  int startTime, endTime;
  while(loops<(time/30))
  {
    distance = watch(); 
    if((distance - prevDistance) > 55)
    {
      startTime = millis();
    }
    else if(((prevDistance - distance) > 55) && ((millis()-startTime) >500))
    {
      parallelPark();
    }
    delay(30);
    prevDistance = distance;
  }
}

void parallelPark()
{
  cornerBack(LEFT);
  delay(500);
  set_Motorspeeds(-200, -200);
  delay(500);
  cornerBack(RIGHT);
  delay(600);
  stop();
  delay(3000);
}

void loop() {
  //auto_navigation();
  //parallelPark();
  // wallDirection = RIGHT;
  // head.write(0);
  // contWallFollowing(500);
}
