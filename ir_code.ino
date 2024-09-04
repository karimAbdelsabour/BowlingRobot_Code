#define IRLEFT 6
#define IRFACE 7
#define IRBACK 8
#define IRRIGHT 9
int L;
//left
int R;
//right
int F;
//forward
int B;
//backward
void setup()
{
  pinMode(IRLEFT, INPUT);
  pinMode(IRFACE, INPUT);
  pinMode(IRBACK, INPUT);
  pinMode(IRRIGHT, INPUT);
  Serial.begin(9600);
}

void loop()
{
  L=digitalRead(IRLEFT);
  R=digitalRead(IRRIGHT);
  F=digitalRead(IRFACE);
  B=digitalRead(IRBACK);
}
void IRFUNC()
{
  //condtion 1
      if(L==1 && F==1){
      Serial.println("go straght");
      delay(5000);
      if(L==1 && F==1 && B==1){
        //we was in C
        Serial.println("we are in D");
      }
      else if(B==1 && L==0 && F==0){
        //we was in B
        Serial.println("we are in A");
      }
    }
    //condtion 2
    else if(R==1 && F==1){
      Serial.println("go straght");
      delay(5000);
      if(R==1 && F==1 && B==1){
        //we was in B
        Serial.println("we are in D");
      }
      else if(B==1 && R==0 && F==0){
        //we was in C
        Serial.println("we are in A");
      }
    }
    //condtion 3
    else if(L==1 && B==1){
      Serial.println("turn left 90 degrees");
      Serial.println("go straght");
      delay(5000);
      if(L==1 && F==1 && B==1){
        //we was in C
        Serial.println("we are in D");
      }
      else if(B==1 && L==0 && F==0){
        //we was in B
        Serial.println("we are in A");
      }
    }
    //condtion 4
    else if(R==1 && B==1){
      Serial.println("turn right 90 degrees");
      Serial.println("go straght");
      delay(5000);
      if(R==1 && F==1 && B==1){
        //we was in B
        Serial.println("we are in D");
      }
      else if(B==1 && R==0 && F==0){
        //we was in C
        Serial.println("we are in A");
      }
    }
}
