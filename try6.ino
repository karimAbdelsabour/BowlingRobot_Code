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

int n_of_wheel_pulses=400; //number of encoder pulses to rtates a complete revolution around itself 400 is just an example example


int n_of_quar_pulses=n_of_wheel_pulses *Drobot/(4.0*Dwheel);//number of encoder pulses to make the robot  rtate a quarter revolution around itself

float perimeter_of_wheel =3.14*Dwheel; //perimeter of the wheel

int stack1 [20];
int stack2 [20];
int top_of_stack = -1;

int flag_going_ahead = 1;


void setup()
{
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
     /*test*/
    rotate_with_number_of_quarters_cloockwise(3);
    // delay(1000);
    rotate_with_number_of_quarters_counter_cloockwise(3);
    //delay(1000);
    rotate_with_distance_forwrd(40);
    //delay(1000);
    rotate_with_distance_backword(40);
    //delay(1000);
    going_back();*/
pinMode(1, OUTPUT);    
digitalWrite(1, HIGH);  

}

void loop()
{
    // Main code loop

}
////////////////////////////////////////////////////////////////////////////////////////////////
/*determine direction & movement */
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

    if(flag_going_ahead) //save the opposite of our povement if we are till going ahead
    {
        top_of_stack++;
        stack1 [top_of_stack]=-motor1_pulses;
        stack2 [top_of_stack]=-motor2_pulses;


    }
    counter1=0;
    counter2=0;

}
/////////////////////////////////////////////////////////////////////////////////////////////////
// going back using stack
void going_back()
{
    flag_going_ahead = 0;
    while(top_of_stack>-1)
    {
        moving(stack1[top_of_stack],stack2[top_of_stack]);
        top_of_stack--;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
// rotating clockwise
void rotate_with_number_of_quarters_cloockwise(int n_of_quarters)   //make the motor rota clockwise until reaching the wanted number of quarters
{
    int n =n_of_quarters*n_of_quar_pulses; //wanted pulses
    moving(n,-n);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
// roting counter clockwise
void rotate_with_number_of_quarters_counter_cloockwise(int n_of_quarters)   //make the motor rota counter clockwise until reaching the wanted number of quarters
{
    int n =n_of_quarters*n_of_quar_pulses; //wanted pulses
    moving(-n,n);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//moving forward
void rotate_with_distance_forwrd(float dis)
{
    int n=(dis/perimeter_of_wheel)*n_of_wheel_pulses; //number of pulses needed to move the distance
    moving(n,n);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//moving backword
void rotate_with_distance_backword(float dis)
{
    int n=(dis/perimeter_of_wheel)*n_of_wheel_pulses; //number of pulses needed to move the distance
    moving(-n,-n);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
