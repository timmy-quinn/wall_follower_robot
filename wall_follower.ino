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

typedef enum scanAngle
{
  rightScan = 0,
  rightDiagonalScan,
  centerScan,
  leftDiagonalScan,
  leftScan
} scanAngle;

int scanValues[5];

enum direction
{
  RIGHT=0,
  LEFT=1, 
  NONE=2
};
direction wallDirection = NONE;
int prevWallDistance = 30;

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
const int distancelimit = 20; //distance limit for obstacles in front           
const int sidedistancelimit = 20; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
const int wallLimit = 25;
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends moving (miliseconds)
const int followingLoopInterval = 10;

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

void go_Forward(int speed, int duration)
{
  int startTime = millis();
  set_Motorspeeds(speed, speed);
  head.write(90);
  while(millis() - startTime < duration)
  {
    if(watch()<=distancelimit)
    {
      stop();
      break;
    }
  }
}

void spin(int direction, int speed)
{
  if(direction==LEFT) set_Motorspeeds(-1*speed, speed*0.75);

  else if (direction==RIGHT) set_Motorspeeds(speed * 0.75, -1*speed);
}

void spin_degrees(int direction, int degrees)
{
  spin(direction, 255);
  delay(degrees*4.3);
}

void corner(int direction)
{
  if (direction == LEFT) set_Motorspeeds(90, 250);
  else if (direction == RIGHT) set_Motorspeeds(250, 90);
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

void printScanValues()
{
  Serial.println("******************************************");
  Serial.println("Scanvalues");
  Serial.print("Right scan value: ");
  Serial.println(scanValues[rightScan]);
  Serial.print("Right diagonal scan value: ");
  Serial.println(scanValues[rightDiagonalScan]);
  Serial.print("Center scan value: ");
  Serial.println(scanValues[centerScan]);
  Serial.print("Left diagonal scan value: ");
  Serial.println(scanValues[leftDiagonalScan]);
  Serial.print("Left scan value: ");
  Serial.println(scanValues[leftScan]);
  Serial.println("******************************************");

}

/*detection of ultrasonic distance: total delay of 20 microseconds*/
long watch(){
  long echo_distance = 0;
  int avg = 0;
  int count = 4;
  for(int i = 0; i < count; i++)
  {
    digitalWrite(Trig_PIN,LOW);
    delayMicroseconds(5);                                                                              
    digitalWrite(Trig_PIN,HIGH);
    delayMicroseconds(15);
    digitalWrite(Trig_PIN,LOW);
    echo_distance=pulseIn(Echo_PIN,HIGH);
    echo_distance=echo_distance*0.01657; //how far away is the object in cm
    //Serial.println((int)echo_distance);
    avg = avg + (echo_distance*0.25);
  }
  return round(avg);
}

void findWall()
{
  head.write(180); 
  delay(300);
  leftscanval = watch();
  head.write(0); 
  delay(100);
  leftscanval = watch();
  if(leftscanval<rightscanval)
  {
    wallDirection = LEFT;
  }
  else wallDirection = RIGHT;
}

//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
void watchSurroundings()
{
  if(wallDirection == RIGHT or wallDirection == NONE)
  {
    for(int i = 0; i < 5; i++)
    {
      head.write(i*45);
      delay(300);
      scanValues[i] = watch();    
    }
  }

  if(wallDirection == LEFT)
  {
    for(int i = 4; i >= 0; i--)
    {
      head.write(i*45);
      delay(300);
      scanValues[i] = watch();    
    }
  }
  head.write(wallDirection * 180);
  delay(500);
}

void chooseDirection()
{
  robot.turn_left = false;
  robot.turn_right = false;
  robot.hard_turn = false;

  if(scanValues[rightScan] < sidedistancelimit || scanValues[rightDiagonalScan] < wallLimit)
  {
    robot.turn_left = true;
  }
  if(scanValues[leftScan] < sidedistancelimit || scanValues[leftDiagonalScan] < wallLimit)
  {
    robot.turn_right = true;
  }
  if(scanValues[centerScan] < wallLimit || scanValues[leftDiagonalScan] < wallLimit || scanValues[rightDiagonalScan] < wallLimit)
  {
    robot.hard_turn = true;
  }
}

void navigateCorner(int time)
{
  go_Forward(FAST_SPEED, 10);

  while()
}

void contWallFollowing(int time)
{
  int loops = 0;
  int offset;
  int distance;
  int prevDistance;
  int rateOfChange;
  int prevOffset = 0;
  while(loops<(time/followingLoopInterval))
    {
      distance = watch();
      rateOfChange = (offset - prevOffset);
      offset = ((distance - wallLimit))*10;
      rateOfChange = offset - prevOffset;
      fitInt(&offset, 62);
      Serial.println(offset);
      Serial.print("Walldirection");
      Serial.println(wallDirection);
      if(offset > 2* wallLimit)
      {
        //spin_degrees(wallDirection, 30);
        corner(wallDirection);
        delay(turntime);
        head.write(90);
        delay(300);
        //go_Forward(255, turntime);
        loops = time/followingLoopInterval;
      }
      if (wallDirection == RIGHT)
      {
        set_Motorspeeds((125 + offset), (125 - offset));
      }
      else if(wallDirection == LEFT)
      {
        set_Motorspeeds((125 - offset), (125 + offset));

      }
      prevOffset = offset;
      delay(10);
      loops++;
    }

}

//Maintain distance from the identified wall
void followWall()
{
  int wallDistance;
  int wallChange;
  if(wallDirection == RIGHT) wallDistance = rightscanval;
  else if (wallDirection == LEFT) wallDistance = leftscanval;
  wallChange = wallDistance - prevWallDistance; 
  
  if (wallDirection == NONE)
  {
    //* go forward
    go_Forward(SPEED, backtime);
    stop();
  }
  // else if (wallDirection == LEFT && leftscanval >= (wallLimit*1.3))
  // {
  //   //turn left
  //   corner(LEFT);
  //   delay(turntime);
  //   stop();
  // }
  else if (wallDirection == LEFT && leftscanval >= wallLimit && wallChange > 30)
  {
    //turn left
    turn(LEFT);
    delay(turntime);
    stop();
  }
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
  else if (wallDirection == RIGHT && rightscanval > wallLimit)
  {
    //turn right
    turn(RIGHT);
    delay(turntime);
    stop();
  }
  else if (wallDirection == RIGHT && wallChange > 30)
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
    go_Forward(SPEED, turntime);
  }
  stop();
}

//navigate through a room, finding and following a wall
void auto_navigation(){

    //Adjust Course
    watchSurroundings();
    chooseDirection();
    if (robot.turn_left && robot.turn_right && robot.hard_turn )
    {
      if(wallDirection == RIGHT) spin(LEFT, TURN_SPEED);
      else spin(RIGHT, TURN_SPEED);
      delay(turntime/2);
      stop();
    }
   
    if (robot.turn_right && robot.hard_turn)
    {
      //spin right
      spin(RIGHT, TURN_SPEED);
      delay(turntime);
      stop();
    }
    else if(robot.turn_left && robot.hard_turn)
    {
      spin(LEFT, TURN_SPEED);
      delay(turntime);
      stop();
    }
    else if (robot.hard_turn)
    {
      if(wallDirection == RIGHT) spin(LEFT, TURN_SPEED);
      else spin(RIGHT, TURN_SPEED);
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
      head.write(wallDirection*180);
      //follow wall
      contWallFollowing(turntime);
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


  digitalWrite(Trig_PIN,LOW);
  /*init servo*/
  head.attach(SERVO_PIN); 
  head.write(90);
   delay(2000);
  
  Serial.begin(9600);
 findWall();
 
}
void findPark(int time)
{
  int loops = 0;
  int leftSpeed = SPEED;
  int rightSpeed = SPEED;
  int offset;
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
  // turn(RIGHT);
  // delay(turntime);
  // stop();
  // delay(5000);
  //auto_navigation();
  corner(LEFT);
  // watchSurroundings();
  // printScanValues();

  //parallelPark();
  //wallDirection = RIGHT;
}
