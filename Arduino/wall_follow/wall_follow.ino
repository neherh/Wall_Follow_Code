//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <Servo.h>


// ROS variables
ros::NodeHandle nh;

// Publishing Encoder values
std_msgs::Float32MultiArray encoder_arr;
std_msgs::Float32 turning;
std_msgs::Bool state;
ros::Publisher chatter ("/wall_follow/odometry", &encoder_arr);
ros::Publisher chatter2("/wall_follow/turn", &turning);

//declare variables
float encoder_value[2] = {0,1};   // variables for putting encoder values in
float traj_angle;
bool states;
int x,start,stp;


// subscriber CallBack
void messageCb (const std_msgs::Float32& trajectory_angle){
  traj_angle = trajectory_angle.data;
}

void stateCb (const std_msgs::Bool& message){
  states = state.data;
}



//initialize subscriber
ros::Subscriber<std_msgs::Float32> sub("/wall_follow/traj_angle", &messageCb);
ros::Subscriber<std_msgs::Bool> sub2("/wall_follow/state", &stateCb);




//__________________Set up encoder stuff_____________________//

enum PinAssignments {
  encoderPinA = 2,   // rigth
  encoderPinB = 3,   // left
  encoderPinC = 18,   // right
  encoderPinD = 19,   // left
};

volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
volatile unsigned int encoderPos1 = 0;  // a counter for the dial
unsigned int lastReportedPos1 = 1;   // change management
static boolean rotating=false;      // debounce management

// interrupt service routine vars
boolean A_set = false;              
boolean B_set = false;
boolean A1_set = false;              
boolean B1_set = false;




//___________Servo stuff_______________//

Servo throttleServo,steerServo;  //output
#define SERVO_THROTTLE_PIN    10
#define SERVO_STEER_PIN       11
int var = 500;




void setup() {
//Serial.begin(9600);
x=1;
stp = 0;

// initialize nodes and stuff

//nh.getHardware()->setBaud(57600);
nh.getHardware()->setBaud(115200);
nh.initNode();
nh.advertise(chatter);
nh.advertise(chatter2);
nh.subscribe(sub);
nh.subscribe(sub2);
 
  
encoder_arr.layout.dim[0].label = "test";
encoder_arr.layout.dim[0].size = 2;
encoder_arr.layout.dim[0].stride = 1*2;
encoder_arr.layout.data_offset = 0;
encoder_arr.data_length = 2;
  
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  pinMode(encoderPinC, INPUT); 
  pinMode(encoderPinD, INPUT);
 // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(encoderPinC, HIGH);
  digitalWrite(encoderPinD, HIGH);

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
// encoder pin on interrupt 3 (pin 18)
  attachInterrupt(3, doEncoderC, CHANGE);
// encoder pin on interrupt 4 (pin 19)
  attachInterrupt(4, doEncoderD, CHANGE);

//_________________Steering & Throttle________________//

  throttleServo.attach(SERVO_THROTTLE_PIN);
  steerServo.attach(SERVO_STEER_PIN);
  
}

//ESC SETUP
void setup_ESC()
{
  long tmp_time = millis();
  while ((millis() - tmp_time) <5000)
    throttleServo.writeMicroseconds(1500);
}

void loop() {

nh.spinOnce();


float turn = (1000/3.14)*traj_angle+1500;
turning.data = turn;

//if (states == true){
//throttleServo.writeMicroseconds (1545);
//}
//if (timer() == 1){
 // Serial.println('true');
  throttleServo.writeMicroseconds (1615);
//} 
//else{
 // Serial.println('faslse');
//  throttleServo.writeMicroseconds (1510);
//}
steerServo.writeMicroseconds (turn);
  
// Finding ticks from encoders

rotating = true;  // reset the debouncer

  if (lastReportedPos != encoderPos) {
    //Serial.print("IndexL:");
    //Serial.println(encoderPos, DEC);
    encoder_value[0] = encoderPos;
    lastReportedPos = encoderPos;
  }
  

  
  if (lastReportedPos1 != encoderPos1) {
    //Serial.print("IndexR:");
    //Serial.println(encoderPos1, DEC);
    encoder_value[1] = encoderPos1;
    lastReportedPos1 = encoderPos1;
  }  
  



// assigning values and publishing to ROS
    encoder_arr.data = encoder_value;         // assigning values from encoder to ROS message
    chatter.publish(&encoder_arr);            // publishing  ROS message to ROS
    chatter2.publish(&turning);
    //delay(1000);

}

// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
      encoderPos += 2;

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 2;

    rotating = false;
  }
}

// Interrupt on A1 changing state
void doEncoderC(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinC) != A1_set ) {  // debounce once more
    A1_set = !A1_set;

    // adjust counter + if C leads D
    if ( A1_set && !B1_set ) 
      encoderPos1 += 2;

    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B1 changing state, same as A above
void doEncoderD(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinD) != B1_set ) {
    B1_set = !B1_set;
    //  adjust counter - 1 if D leads C
    if( B1_set && !A1_set ) 
      encoderPos1 += 2;

    rotating = false;
  }
}

int timer(){
  if (x == 1 && stp == 0){
    start = millis();
    stp = start;
    return 1;
  }
  else if(x == 0){
    start = millis();
    if (start - stp >= var+200){
      x = 1;
      stp = start;
      return 1;
    }
    else{
      return 0;
    }
  }
  else if(x == 1){
    start = millis();
    if(start - stp >= var){
      x = 0;
      stp = start;
      return 0;
    }
    else{
      return 1;
    }
  }
}

