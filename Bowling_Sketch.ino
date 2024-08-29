//////////////////////////////////////////////////////////////////////////////////////////////// Motor pins
#define ENCA1 2  //encoder motor 1
#define ENCB1 A0
#define ENCA2 3  //encoder motor 2
#define ENCB2 A1
#define ENA   10  // control the speed of motor 1
#define IN1   9  // first input of motor 1
#define IN2   8  // second input of motor 1
#define ENB   11 // control the speed of motor 2
#define IN3   12  // first input of motor 2
#define IN4   13   // second input of motor 2
#define speed 50 //speed configeration

int counter1 = 0; //encoder 1 counting
int counter2 = 0; //encoder 2 counting
int Drobot=20; //robot diametwer
int Dwheel=5; //wheel diametwer
int n_of_wheel_pulses=400; //number of encoder pulses to rotates a complete revolution around itself 400 is just an example example
int n_of_quar_pulses=n_of_wheel_pulses *Drobot/(4.0*Dwheel);//number of encoder pulses to make the robot  rotate a quarter revolution around itself
float perimeter_of_wheel = 3.14*Dwheel; //perimeter of the wheel

/*
int stack1 [20];
int stack2 [20];
int top_of_stack = -1;
int flag_going_ahead = 1;
*/

//////////////////////////////////////////////////////////////////////////////////////////////// Motor pins
//////////////////////////////////////////////////////////////////////////////////////////////// IR Module Pins
int Left;
int Right;
int Front;
int Back;
#define IR_LEFT 6
#define IR_FACE 7
#define IR_BACK 8
#define IR_RIGHT 9
//////////////////////////////////////////////////////////////////////////////////////////////// IR Module Pins
//////////////////////////////////////////////////////////////////////////////////////////////// TOF Pins
#include <Wire.h>
#include <VL53L1X.h>
VL53L1X TOF_Sensor;
//////////////////////////////////////////////////////////////////////////////////////////////// TOF Pins
//////////////////////////////////////////////////////////////////////////////////////////////// Cannon & Relay Pins
#include <Servo.h>
#define  controlPin 7

Servo Ball_Servo;
//////////////////////////////////////////////////////////////////////////////////////////////// Cannon & Relay Pins
//////////////////////////////////////////////////////////////////////////////////////////////// Functions Declarations
void countEncoder1();
void countEncoder2();
void rotate_with_number_of_quarters_cloockwise(int n_of_quarters);
void rotate_with_number_of_quarters_counter_cloockwise(int n_of_quarters);
void Move_with_distance_forwrd(float dis);
void Move_with_distance_backword(float dis);
void Detecting_Location(int Left, int Right, int Front, int Back);
void moving(int motor1_pulses, int motor2_pulses);
void Open_The_Door_For_Ball();
void Enabling_Relay_TO_555_TIMER();
void TOF_Detecting();
//void going_back();
//////////////////////////////////////////////////////////////////////////////////////////////// Functions Declarations
void setup()
{
//////////////////////////////////////////////////////////////////////////////////////////////// IR Module Setup
     pinMode(IR_LEFT, INPUT);
     pinMode(IR_FACE, INPUT);
     pinMode(IR_BACK, INPUT);
     pinMode(IR_RIGHT, INPUT);
     Serial.begin(9600);
//////////////////////////////////////////////////////////////////////////////////////////////// IR Module Setup
//////////////////////////////////////////////////////////////////////////////////////////////// Motor setup
   // Motor 1 pins direction
     pinMode(ENA, OUTPUT);
     pinMode(IN1, OUTPUT);
     pinMode(IN2, OUTPUT);
    // Motor 2 pins direction
     pinMode(ENB, OUTPUT);
     pinMode(IN3, OUTPUT);
     pinMode(IN4, OUTPUT);
    // Encoder 1 pins direction
     pinMode(ENCA1, INPUT_PULLUP);
     pinMode(ENCB1, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(ENCA1), countEncoder1, RISING);//interrupt when first input of motor 1 rising for counting
    // Encoder 2 pins direction
     pinMode(ENCA2, INPUT_PULLUP);
     pinMode(ENCB2, INPUT_PULLUP);
     attachInterrupt(digitalPinToInterrupt(ENCA2), countEncoder2, RISING); //interrupt when first input of motor 1 rising for counting
//////////////////////////////////////////////////////////////////////////////////////////////// Motor setup
//////////////////////////////////////////////////////////////////////////////////////////////// Ball_Servo & Realy_555_timer Setup
    digitalWrite(controlPin, LOW);
    Ball_Servo.attach(9);
//////////////////////////////////////////////////////////////////////////////////////////////// Ball_Servo & Realy_555_timer Setup
//////////////////////////////////////////////////////////////////////////////////////////////// TOF Sensor Setup
     Wire.begin();
     TOF_Sensor.setTimeout(500);
     TOF_Sensor.init();                                      // Initialize the TOF
     TOF_Sensor.setROISize(4, 4);                            // set ROI to minimum FOV (4x4) 15 degree
     TOF_Sensor.setDistanceMode(VL53L1X::Short);             // Change to Short mode for up to 1.3m
     TOF_Sensor.setMeasurementTimingBudget(20000);           // set Time Budget to 20ms
//////////////////////////////////////////////////////////////////////////////////////////////// TOF Sensor Setup
}
//////////////////////////////////////////////////////////////////////////////////////////////// Main Loop
void loop() 
{

  Left=digitalRead(IR_LEFT);
  Right=digitalRead(IR_RIGHT);
  Front=digitalRead(IR_FACE);
  Back=digitalRead(IR_BACK);
  Detecting_Location( Left , Right , Front , Back );
  TOF_Detecting();
  Enabling_Relay_TO_555_TIMER();
  Open_The_Door_For_Ball();
  rotate_with_number_of_quarters_counter_cloockwise(2);  
  Move_with_distance_forwrd(22);
  
}
//////////////////////////////////////////////////////////////////////////////////////////////// Main Loop
//////////////////////////////////////////////////////////////////////////////////////////////// Location Detection Function Block

void Detecting_Location(int Left, int Right, int Front, int Back) {
  // First Condition -------> Two Cases are handled here, When our Robot in B Looking to the front and When our Robot in C Looking to the Right
  if (Left == 1 && Front == 1) {
    Serial.println("Move straight 22cm");
    Move_with_distance_forwrd(22);
    delay(1000);

    if (Left == 1 && Front == 1 && Back == 1) { // we were in C
      Serial.println("we are in D from C so rotate and move forward 22cm");
      rotate_with_number_of_quarters_counter_cloockwise(1);
      Move_with_distance_forwrd(22);

      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("We Are in A");
      delay(1000);
      Move_with_distance_forwrd(7); // we modify the Robot's position to be in front of the pins
    } else if (Back == 1 && Left == 0 && Front == 0) { // we were in B
      Serial.println("we are in A");
      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("we are in A");
      delay(1000);
      rotate_with_number_of_quarters_counter_cloockwise(1); // we modify the Robot's position to be in front of the pins
      delay(1000);
      Move_with_distance_forwrd(20);
      delay(1000);
      rotate_with_number_of_quarters_cloockwise(1);
      Serial.println("We Are in Front of the Pins");
    }
  }

  // Second Condition -------> Two Cases are handled here, When our Robot in B Looking to the Right and When our Robot in C Looking to the Front
  else if (Right == 1 && Front == 1) {
    Serial.println("Move straight 22cm");
    Move_with_distance_forwrd(22);
    delay(1000);

    if (Right == 1 && Front == 1 && Back == 1) { // we were in B
      Serial.println("we are in D from B so rotate and move forward 22cm");
      rotate_with_number_of_quarters_cloockwise(1);
      Move_with_distance_forwrd(22);

      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("We Are in A");
      delay(1000);
      Move_with_distance_forwrd(7); // we modify the Robot's position to be in front of the pins
    } else if (Back == 1 && Right == 0 && Front == 0) { // we were in C
      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("we are in A");
      delay(1000);
      rotate_with_number_of_quarters_cloockwise(1); // we modify the Robot's position to be in front of the pins
      delay(1000);
      Move_with_distance_forwrd(20);
      delay(1000);
      rotate_with_number_of_quarters_counter_cloockwise(1);
      Serial.println("We Are in Front of the Pins");
    }
  }

  // Third Condition -------> Two Cases are handled here, When our Robot in C Looking to the Back and When our Robot in B Looking to the Right
  else if (Left == 1 && Back == 1) {
    Serial.println("Rotate 90 degrees Counter Clockwise");
    rotate_with_number_of_quarters_counter_cloockwise(1);
    delay(1000);
    Serial.println("Move straight 22cm");
    Move_with_distance_forwrd(22);
    delay(1000);

    if (Left == 1 && Front == 1 && Back == 1) { // we were in C
      Serial.println("we are in D from C so rotate and move forward 22cm");
      rotate_with_number_of_quarters_counter_cloockwise(1);
      Move_with_distance_forwrd(22);

      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("We Are in A");
      delay(1000);
      Move_with_distance_forwrd(7); // we modify the Robot's position to be in front of the pins
    } else if (Back == 1 && Left == 0 && Front == 0) { // we were in B
      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("we are in A");
      delay(1000);
      rotate_with_number_of_quarters_counter_cloockwise(1); // we modify the Robot's position to be in front of the pins
      delay(1000);
      Move_with_distance_forwrd(20);
      delay(1000);
      rotate_with_number_of_quarters_cloockwise(1);
      Serial.println("We Are in Front of the Pins");
    }
  }

  // Fourth Condition -------> Two Cases are handled here, When our Robot in B Looking to the Back and When our Robot in C Looking to the Left
  else if (Right == 1 && Back == 1) {
    Serial.println("Rotate 90 degrees Clockwise");
    rotate_with_number_of_quarters_cloockwise(1);
    delay(1000);
    Serial.println("Move straight 22cm");
    Move_with_distance_forwrd(22);
    delay(1000);

    if (Right == 1 && Front == 1 && Back == 1) { // we were in B
      Serial.println("we are in D from B so rotate and move forward 22cm");
      rotate_with_number_of_quarters_cloockwise(1);
      delay(1000);
      Move_with_distance_forwrd(22);

      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("We Are in A");
      delay(1000);
      Move_with_distance_forwrd(7); // we modify the Robot's position to be in front of the pins
    } else if (Back == 1 && Right == 0 && Front == 0) { // we were in C
      do {
        Move_with_distance_forwrd(0.5);
      } while (Back == 1 && Left == 0 && Right == 0 && Front == 0); // we make sure that the Robot leaves completely to the serving Area A

      Serial.println("we are in A");
      delay(1000);
      rotate_with_number_of_quarters_cloockwise(1); // we modify the Robot's position to be in front of the pins
      delay(1000);
      Move_with_distance_forwrd(20);
      delay(1000);
      rotate_with_number_of_quarters_counter_cloockwise(1);
      Serial.println("We Are in Front of the Pins");
    }
  }
} 

//////////////////////////////////////////////////////////////////////////////////////////////// Location Detection Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Function Block
void moving(int motor1_pulses,int motor2_pulses)
{
         analogWrite(ENA, 0);
         analogWrite(ENB, 0);
    //direction of motor1
    if(motor1_pulses>0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    //direction of motor2
    if(motor2_pulses>0)
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    else
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    // amount of movement
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    while(abs(motor1_pulses)>abs(counter1))
    {}
       analogWrite(ENA, 0);
       analogWrite(ENB, 0);
/*
    if(flag_going_ahead) //save the opposite of our povement if we are till going ahead
    {
        top_of_stack++;
        stack1 [top_of_stack]=-motor1_pulses;
        stack2 [top_of_stack]=-motor2_pulses;
    }
    counter1=0;
    counter2=0;
*/
}
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Main Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Returning Back Function Block
/*
void going_back()   // going back using stack
{
    flag_going_ahead = 0;
    while(top_of_stack>-1)
    {
        moving(stack1[top_of_stack],stack2[top_of_stack]);
        top_of_stack--;
    }
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////// Returning Back Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Rotating clockwise Function Block
void rotate_with_number_of_quarters_cloockwise(int n_of_quarters)   //make the motor rota clockwise until reaching the wanted number of quarters
{
    int n =n_of_quarters*n_of_quar_pulses; //wanted pulses
    moving(n,-n);
}
//////////////////////////////////////////////////////////////////////////////////////////////// Rotating clockwise Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Rotating counter clockwise Function Block
void rotate_with_number_of_quarters_counter_cloockwise(int n_of_quarters)   //make the motor rota counter clockwise until reaching the wanted number of quarters
{
    int n =n_of_quarters*n_of_quar_pulses; //wanted pulses
    moving(-n,n);
}
//////////////////////////////////////////////////////////////////////////////////////////////// Rotating counter clockwise Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Forward with distance Function Block
void Move_with_distance_forwrd(float dis)
{
    int n=(dis/perimeter_of_wheel)*n_of_wheel_pulses; //number of pulses needed to move the distance
    moving(n,n);
}
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Forward with distance Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Backward with distance Function Block
void Move_with_distance_backword(float dis)
{
    int n=(dis/perimeter_of_wheel)*n_of_wheel_pulses; //number of pulses needed to move the distance
    moving(-n,-n);
}
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Backward with distance Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// encoder 1 counting Function Block Called by Interrupt
//encoder counting
void countEncoder1()
{
    int b = digitalRead(ENCB1);
    if (b > 0)
    {
        counter1++;
    }
    else
    {
        counter1--;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////// encoder 1 counting Function Block Called by Interrupt
//////////////////////////////////////////////////////////////////////////////////////////////// encoder 2 counting Function Block Called by Interrupt
void countEncoder2()
{
    int b = digitalRead(ENCB2);
    if (b > 0)
    {
        counter2++;
    }
    else
    {
        counter2--;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////// encoder 2 counting Function Block Called by Interrupt
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Ball Servo Function Block
void Open_The_Door_For_Ball()
{
  Ball_Servo.write(180);
  delay(10000);  
}
//////////////////////////////////////////////////////////////////////////////////////////////// Moving Ball Servo Function Block
//////////////////////////////////////////////////////////////////////////////////////////////// Moving the Rollers  Function Block
void Enabling_Relay_TO_555_TIMER()
{
  digitalWrite(controlPin, HIGH);
  delay(4000); 
}
//////////////////////////////////////////////////////////////////////////////////////////////// Moving the Rollers  Function Block
////////////////////////////////////////////////////////////////////////////////////////////////  TOF Sensor Detecting Function Block

void TOF_Detecting() {
    TOF_Sensor.startContinuous(20); // Set the inter-measurement period to 20 ms

    float minDistance1 = 114.0;  
    float minDistance2 = 114.0;  
    float currentDistance;
    
    float increment = 0.05; // Increment value for rotations
    float maxRotation = 0.2; // Maximum quarters to rotate
    float currentRotation1 = 0.0; // Start from 0
    float currentRotation2 = 0.0; // Start from 0
    float angle = 20; // Angle increment for readings

    while (currentRotation1 < maxRotation) {
        currentDistance = TOF_Sensor.read(); 

        if (currentDistance < minDistance1) {        // Check if the current distance is the new minimum

            minDistance1 = currentDistance;
        }
        rotate_with_number_of_quarters_cloockwise(increment); 
        delay(1000); 

        currentRotation1 += increment; // Increment the current rotation
    }


    while (currentRotation2 < maxRotation) {
        currentDistance = TOF_Sensor.read(); 

        if (currentDistance < minDistance2) {        // Check if the current distance is the new minimum

            minDistance2 = currentDistance;
        }
        rotate_with_number_of_quarters_counter_cloockwise(increment); 
        delay(1000); 

        currentRotation2 += increment; // Increment the current rotation
    }

    if(minDistance1 > minDistance2) 
      rotate_with_number_of_quarters_counter_cloockwise(currentRotation2);
    else
      rotate_with_number_of_quarters_cloockwise(currentRotation1); 
}
////////////////////////////////////////////////////////////////////////////////////////////////  TOF Sensor Detecting Function Block


