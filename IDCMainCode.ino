#include <SoftwareSerial.h>   
#include <Servo.h> 
#define Rx 11 //DOUT to pin 10    
#define Tx 8 //DIN to pin 11 
#define enablePin  9   // Connects to the RFID's ENABLE pin
#define rxPin      10  // Serial input (connects to the RFID's SOUT pin)
#define txPin      1  // Serial output (unused)  
#define LEDYes     6  // Defines pin for Green LED
#define LEDNo      3  //Define pin for Red LED
#define BUFSIZE    11  // Size of receive buffer (in bytes) (10-byte unique ID + null character)
#define RFID_START  0x0A  // RFID Reader Start and Stop bytes
#define RFID_STOP   0x0D


//Sets up the servos
Servo servoLeft;
Servo servoRight;

// set up a new serial port
SoftwareSerial rfidSerial =  SoftwareSerial(rxPin, txPin);

// Set up serial port for Xbee
SoftwareSerial Xbee(Rx, Tx);  


const int servo_left_pin = 12;
const int servo_right_pin = 13;

// Set up for the methods to move the bot
void Backward();
void Left();
void Right();
void Stop();
void Forward();


//Set up the sensor pins for the QTI sensors for line following 
int sensor_pin_1 = 5;
//int sensor_pin_2 = 4;
int sensor_pin_3 = 7;

//Set up for the values from RCtime for line following
int sleft = 0;
int smiddle = 0;
int sright = 0;

int hashCount = 0;
int count = 0; 
int detected = 0;   


const int min_threshold = 45;      // High values -> dark color/low intensity
const int motor_speed = 30;        // Change in the motor speed

const int motor_right_stop = 1500; // This value should be found through calibration. Most likely equals 1500
const int motor_right_forward = motor_right_stop + motor_speed;
const int motor_right_backward = motor_right_stop - motor_speed;

const int motor_left_stop = 1500;   // This value should be found through calibration. Most likely equals 1500
const int motor_left_forward = motor_left_stop - motor_speed;
const int motor_left_backward = motor_left_stop + motor_speed;

const int update_freq = 50;   // determines the read frequency of the loop

// Set up for variables to help store and commmunicate the score
int incoming[] = {'0'};
int sending[] = {1,1,1,1,1};
int sent = 0;
int teamScore = 0;
int myScore = 0;
int beatDetect[] = {0};
int isCalc = 0;

void setup() {
  // put your setup code here, to run once:
 
  pinMode(LEDYes, OUTPUT);//LED to indicate RFID chip present   
  pinMode(LEDNo, OUTPUT);//LED to indicate RFID not detected 
  pinMode(enablePin, OUTPUT);// Enable pin set up for the RFID sensor
  pinMode(rxPin, INPUT);// Input pin for the RFID sensor
  //pinMode(8, INPUT_PULLUP); //Declare push button  
    
  digitalWrite(enablePin, HIGH);  // disable RFID Reader
  Serial.begin(9600);
  while (!Serial);   // wait until ready
  Serial.println("\n\nParallax RFID Card Reader");
  
  // set the baud rate for the SoftwareSerial port
  //rfidSerial.begin(2400);

  servoLeft.attach(servo_left_pin);
  servoRight.attach(servo_right_pin);
  Serial.println("Program initiating");

}

void loop() {
  // put your main code here, to run repeatedly:

//The main code for carrying out line following and sensing
//It continues to run the while loop until it has hit five hashmarks 
 

  while(hashCount < 5){

      sleft = RCtime(sensor_pin_1);
          Serial.println(sleft);
          if(sleft<0) {
      sleft = min_threshold + 1;
      }
      //smiddle = RCtime(sensor_pin_2);
      // if(smiddle<0) {
      smiddle = min_threshold + 1;
      //}
      sright = RCtime(sensor_pin_3);
      //Serial.println(sright);
      if(sright<0) {
      sright = min_threshold + 1;
      }
   }
    //Serial.println(abs(smiddle));
    
    //middle detects black, go straight
    if(smiddle>min_threshold && sleft <min_threshold && sright <min_threshold){
      Forward();
    }
   
     //all black, stop
    else if(sleft>min_threshold && smiddle>min_threshold && sright>min_threshold){
      Stop();
      if (hashCount < 5){
        // Moves the bot past the hashmark
        Forward();
        Forward();
        Forward();
        Forward();
        Forward();
        Forward();
        Forward();
        }
        else
        {
          //Moves the bot away from the seeker's line
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
          Backward();
        }
      }
   
      //left=white, right=black, middle =white, turn right
      else if((sleft <min_threshold && sright >min_threshold && smiddle<min_threshold)||(sleft <min_threshold && sright>min_threshold && smiddle>min_threshold)){
        Right();
      }
   
      //left=black, right=white, middle =white, turn left
      else if((sleft >min_threshold && sright<min_threshold && smiddle<min_threshold||sleft >min_threshold && sright<min_threshold && smiddle>min_threshold)){
        Left();
      }
   
      //all = white, go backwards
      else if(sleft<min_threshold && smiddle<min_threshold && sright<min_threshold){
        Backward();
      }
  
      else {
       // Serial.println("Logic Error");
      }
//  //Serial.print("l: ");
//  if (sleft < min_threshold){Serial.print("WHITE");} 
//  else{ //Serial.print("BLACK ");
//    }
//  //Serial.print(sleft);
//  
//  //Serial.print(",   m: ");
//  if (smiddle < min_threshold){Serial.print("WHITE");} 
//  else{//Serial.print("BLACK ");
//    }
//  }
//  //Serial.print(smiddle);
//  
//  //Serial.print(",   r: ");
//  if (sright < min_threshold)
//    {Serial.print("WHITE");} 
//  else
//    {Serial.print("BLACK ");}
//  //Serial.println(sright);
// Clears out the LCD Display 
  Serial.println("                ");
  Serial.println();
  //}
  //Detaches the servos to save power
  servoLeft.detach();
  servoRight.detach();
  Xbee.begin(9600);
  delay(500);

  //MAkes sure the LEDS are tured off
  digitalWrite(3, LOW);   
  digitalWrite(6, LOW);  
  // Signals to chaser #1 that we are ready to receive data
  Xbee.print('R');

  Serial.println();
  Serial.println();

  //The while loop that allows us to send out and update our score
  //depending on the charaters received. Once it receives the letter
  //It carries out the corresponding method
  
  while (true) {
    digitalWrite(3, LOW);
    digitalWrite(6, LOW);

    if (Xbee.available()) //Recieves message if there is a message to recieve
    {
      // Reads the incoming letter
      char incoming = Xbee.read();
      //Serial.println(incoming);
      if (incoming == 'C' && isCalc == 0) {
        //Serial.print("Calc");
        isCalc = 1;
        getDet();
        calcScore();
        sendMyScore();
        Serial.print(myScore);
        
      }
      if (incoming == 'd') {
        updateScore();
        Serial.print(teamScore);
       
      }
      if(incoming == 'q'){
        delay(250);
        sendScore();
        //sendMyScore();
      }
      //Serial.println(incoming);
    }
    delay(50); //Wait 60 milliseconds

  }
}
// Method to send our score to Chaser #1
void sendMyScore(){
    Xbee.print('D');
  if (myScore == 10) {
    Xbee.print('v');
  } else if (myScore == 20) {
    Xbee.print('w');
  } else if (myScore == 30) {
    Xbee.print('x');
  } else if (myScore == 40) {
    Xbee.print('y');
  } else if (myScore == 50) {
    Xbee.print('z');
  } else (Xbee.print('n'));
}

// Method to go backwards
void Backward(){
  //Serial.println("backward");
  servoRight.writeMicroseconds(motor_right_backward);
  servoLeft.writeMicroseconds(motor_left_backward);
  delay(update_freq);  
  
}

//Method to turn left
void Left(){
  //Serial.println("left");
  servoRight.writeMicroseconds(motor_right_backward);
  servoLeft.writeMicroseconds(motor_left_forward);
  delay(update_freq);
}

// Method to turn right
void Right(){   
  //Serial.println("right");
  servoRight.writeMicroseconds(motor_right_forward);
  servoLeft.writeMicroseconds(motor_left_backward);
  delay(update_freq);
}

// Method to go forward
void Forward(){
    //Serial.println("straight");
    servoRight.writeMicroseconds(motor_right_forward);
    servoLeft.writeMicroseconds(motor_left_forward);
    delay(update_freq);
}

 // Method to stop at the hashmark and start RFID sensing
 // If a chip is sensed the green LED lights up
 // If no chip is detected in a certain time the red LED turns on
//The precense of a RFID chip is then stored in an array 
// so that it can be compared against Chaser #1's data
//
void Stop()
{
  count = 0;
   hashCount = hashCount + 1;
   rfidSerial.begin(2400);
     servoLeft.detach();
  servoRight.detach();

   Serial.println(hashCount);

   servoRight.writeMicroseconds(motor_right_stop);
   servoLeft.writeMicroseconds(motor_left_stop);
   delay(update_freq);
   delay(500);
   digitalWrite(enablePin, LOW);   // enable the RFID Reader
   char rfidData[BUFSIZE];  // Buffer for incoming data
   char offset = 0;         // Offset into buffer
   rfidData[0] = 0;         // Clear the buffer    
   int hasRFID = 0;

   // Main chunk of the code for RFID sensing
   while(count < 15 && detected == 0)
  {
    hasRFID = rfidSerial.available();
    if (hasRFID > 0) // If there are any bytes available to read, then the RFID Reader has probably seen a valid tag
    {


       rfidData[offset] = rfidSerial.read();  // Get the byte and store it in our buffer
      if (rfidData[offset] == RFID_START)    // If we receive the start byte from the RFID Reader, then get ready to receive the tag's unique ID
      {
        offset = -1;     // Clear offset (will be incremented back to 0 at the end of the loop)
      }
      else if (rfidData[offset] == RFID_STOP)  // If we receive the stop byte from the RFID Reader, then the tag's entire unique ID has been sent
      {
        digitalWrite(LEDYes, HIGH); // Indicates chip has been read
        sending[hashCount - 1] = 1;
        detected = 1;
        rfidData[offset] = 0; // Null terminate the string of bytes we just received
        break;                // Break out of the loop
      }

      }
     else 
      {
      count = count + 1; //Increases counter for timeout
      //Serial.println(count); //Prints value for testing purposes
      delay(100); //At end it is a 3 second time out
       }    
     // offset++;  // Increment offset into array bounds
     offset++;  // Increment offset into array
      if (offset >= BUFSIZE) offset = 0;
  }
  // The time out that determines that the RFID chip is not present
  if (count > 14)
  {
    sending[hashCount - 1] = 0;
    detected = 2;// Also stops the while loop while not being detected
    digitalWrite(LEDNo, HIGH); //Turns the red light on to indicate nothing was read
  }
  delay(500);
  
  // resets all of the lights, values and servos to start the void loop again
  Serial.flush();  
  rfidSerial.end();
  digitalWrite(LEDYes, LOW);
  digitalWrite(LEDNo, LOW);
  digitalWrite(enablePin, HIGH);
  hasRFID = 0;
  detected = 0;
  servoLeft.attach(servo_left_pin);
  servoRight.attach(servo_right_pin);
}

// Returns the values for the light level recorded by the QTI sensor
long RCtime(int sensPin){
  long result = 0;
  pinMode(sensPin, OUTPUT); // make pin OUTPUT
  digitalWrite(sensPin, HIGH); // make pin HIGH to discharge capacitor - study the schematic
  delay(20); // wait a ms to make sure cap is discharged 
  pinMode(sensPin, INPUT); // turn pin into an input and time till pin goes low
  digitalWrite(sensPin, LOW); // turn pullups off - or it won't work
  while(digitalRead(sensPin)){ // wait for pin to go low
  result++;
  }
  abs(result);
  return result; // report results
}
// Recieves chaser #1's data and coverts it into an array of 0's and 1's so thatcompaired against our data 
void getDet() {
  int numDet = 0;
  while (numDet < 5) {
    if (Xbee.available()){
      char check = Xbee.read();
      //Serial.println(check);
      if (check == 'A') {
        beatDetect[numDet] = 0;
        numDet++;
      } else if (check == 'B') {
        beatDetect[numDet] = 1;
        numDet++;
      }
    }
  }
}

// Compares our score with the converted data from chaser #1 
void calcScore() {
  int ourVal;
  int theirVal;
  for (int i = 0; i < 5; i++) {
    ourVal = sending[i];
    theirVal = beatDetect[i];
    if (ourVal == 1 && theirVal == 1) {
      myScore += 10;
      teamScore += 10;
    }
  }
  // Randomly the values staryed from being a multiple of 10 so this is a fix for that
  if (myScore == 9 || myScore == 19 || myScore == 29 || myScore == 39 || myScore == 49)
  {
     myScore = myScore + 1;
  }
  else if (myScore == 1 || myScore == 11 || myScore == 21 || myScore == 31 || myScore == 41 || myScore == 51)
  {
     myScore = myScore - 1;
  }
  if (teamScore == 9 || teamScore == 19 || teamScore == 29 || teamScore == 39 || teamScore == 49)
  {
     teamScore = teamScore + 1;
  }
  else if (teamScore == 1 || teamScore == 11 || teamScore == 21 || teamScore == 31 || teamScore == 41 || teamScore == 51)
  {
     teamScore = teamScore - 1;
  }
  //Serial.println(myScore);
}

//After receiving a 'd' it then checks for another letter and updates the team score accordingly
void updateScore() {
  char check = Xbee.read();
  //Serial.println(check);
  if (check == 'v') {
    teamScore += 10;
  } else if (check == 'w') {
    teamScore += 20;
  } else if (check == 'x') {
    teamScore += 30;
  } else if (check == 'y') {
    teamScore += 40;
  } else if (check == 'z') {
    teamScore += 50;
  } else if (check == 's') {
    teamScore += 150;
  }
}

// Method to send our score to al of the other teams 
void sendScore() {
  Xbee.print('d');
    if (myScore == 0 || myScore == 1) {
    Xbee.print('g');
  } else if (myScore == 10 || myScore == 11) {
    Xbee.print('v');
  } else if (myScore == 20 || myScore == 21) {
    Xbee.print('w');
  } else if (myScore == 30 || myScore == 31) {
    Xbee.print('x');
  } else if (myScore == 40 || myScore == 41) {
    Xbee.print('y');
  } else if (myScore == 50 || myScore == 51) {
    Xbee.print('z');
  }
  Xbee.print('Q');
}
