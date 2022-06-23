int mode = 0;
int dir=0;
# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2
#define LEFT_FOUND 3
#define RIGHT_FOUND 4
#define lb_pressed digitalRead(10)==LOW
#define mb_pressed digitalRead(9)==LOW
#define BUZ_HI digitalWrite(11,HIGH)
#define BUZ_LOW digitalWrite(11,LOW)
int led=8;
const int power = 500;
const int iniMotorPower = 250;
const int adj = 1;
float adjTurn = 8;
int trigpin_frd = 44;    //Trig 
int echopin_frd = 46;    //Echo//middle sonar // FOR UP SONAR

int trigpin_lft = 48;    //Trig FOR LEFT SONAR
int echopin_lft = 50;
int left_speed=150;
int right_speed=150;
int left_speedd,right_speedd;
int trigpin_rght=40;
int echopin_rght=42;//for right sonar

float duration_rght,length_rght,cm_rght,r;//variable for right sonar
float duration_lft,length_lft,cm_lft,l;//variable for left sonar
float duration_frd,length_frd,cm_frd,f;
float duration_up_frd,length_up_frd,cm_up_frd,m;
#define S0 A10
#define S1 A11
#define S2 A9
#define S3 A8
#define sensorOut A12
int b1,b2;
int n1=2,n2=3,n3=5,n4=6,e1=4,e2=7;    //for motor driver
int s1,s2,s3,s4,s5,s6,s7;
// LFSensor more to the Left is "0"
const int lineFollowSensor0 = 38; 
const int lineFollowSensor1 = 36; 
const int lineFollowSensor2 = 34; 
const int lineFollowSensor3 = 32;
const int lineFollowSensor4 = 26;
const int lineFollowSensor5 = 24;
const int lineFollowSensor6 = 22;
const int left_back=30;
const int right_back=28;
int sr=0,su=0,sl=0;
int S[7]={0, 0, 0, 0, 0, 0, 0};

// PID controller
float Kp=65;
float Ki=0;
float Kd=45;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;
int left_value=0;
int right_value=0;
void setup()
{
  Serial.begin(9600);
  pinMode(trigpin_lft, OUTPUT);
  pinMode(echopin_lft, INPUT);//for left sonar
  pinMode(trigpin_rght, OUTPUT);
  pinMode(echopin_rght, INPUT);//for right sonar
  
  pinMode(trigpin_frd, OUTPUT);
  pinMode(echopin_frd, INPUT);//for forward sonar
  pinMode(led,OUTPUT);
  pinMode(n1,OUTPUT);
  pinMode(n2,OUTPUT);
  pinMode(n3,OUTPUT);
  pinMode(n4,OUTPUT);
  pinMode(e1,OUTPUT);
  pinMode(e2,OUTPUT);
  while(lb_pressed)
  {
    delay(10);
  }
  BUZ_HI;
  delay(250);
  BUZ_LOW;
  delay(250);
}
void loo()
{
  l=left_sonar_read();
  Serial.print("l:");
  Serial.print(l);
  Serial.print("  ");
  r=right_sonar_read();
  Serial.print("r:");
  Serial.print(r);
  Serial.print("  ");
  f=forward_sonar_read();
  Serial.print("f:");
  Serial.print(f);
  Serial.println();
  delay(250);
}
/////////////////////////////////////////
void loop() 
{
    check_mode();    
    switch(mode)
   {
    case STOPPED: 
      read_sensor();
      if((s1+s2+s3+s4+s5+s6+s7)==0)
      {
        r=right_sonar_read();
        if(r>0 && r<=20)
        {
          bot(-100,-100);//indicate cave follow
          delay(20);
          BUZ_HI;
          bot(0,0);
          delay(100);
          wall_follow();
         // check();
        }
        else
        {
          bot(200,200);
          delay(180);//some increase forward
          back_turn();
        }
      }
      else
      {
        bot(100,100);
        check_mode();
      }
      previousError = error;
      break;
///////////////////////////////////////////////////////////////////
    case LEFT_FOUND:  
      b1=0;
      while(b1==0)
      {
        bot(200,200);
        read_all_sensor();
      }
       read_sensor(); 
        if((s1+s2+s3+s4+s5+s6+s7)==7)
        {
          bot(0,0);
          digitalWrite(led,HIGH);
          BUZ_HI;
          end_found();
        }
        else
        {
          left_move();
        }
      break;
///////////////////////////////////////////////////////////////////      
    case RIGHT_FOUND: 
      b2=0;
      while(b2==0)
      {
        bot(200,200);
        read_all_sensor();
        if((s1+s2)>=1)
        left_value=1;
      }
      if(left_value==1)// T section found
      {
        if((s1+s2+s3+s4+s5+s6+s7)==7)
        {
          bot(0,0);
          digitalWrite(led,HIGH);
          BUZ_HI;
          end_found();
        }
        else
        {
          left_move();
          left_value=0;
        }
      }
      else
      {
        read_sensor();
        if((s3+s4+s5)>=1)
        {
          bot(200,200);
          delay(70);
        }
        else
        {
          right_move();
        }
      }
      break;

    case FOLLOWING_LINE:    
        calculatePID();
        pid_work();   
      break;     
  }
}

//===========================================================================//
void exact_position()
{
  int exact=1;
  while(exact==1)
  {
    f=forward_sonar_read();
    if(f>0 && f<=7)
    {
      bot(0,0);
      f=forward_sonar_read();
      if(f>0 && f<=9)
      {
        bot(0,0);
        l=left_sonar_read();
        r=right_sonar_read();
        if(l>0 && l<=20 && r==0)// left_move
        {
          bot(-250,250);
          delay(300);
          bot(250,-250);
          delay(50);
          bot(0,0);
          while(1);
        }
        else if(r>0 && r<=20 && l==0)// right_move
        {
          bot(250,-250);
          delay(300);
          bot(-250,250);
          delay(50);
          bot(0,0);
          while(1);
        }  
        else
        bot(100,100);
      }  
      else
      {
        bot(100,100);
      }
    }
    else
    {
      bot(100,100);
    }
  }
}
//==========================================================//
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  previousError = error;
}
/////////////////////////////////////////
void read_sensor()
{
  if(digitalRead(lineFollowSensor0)==HIGH)
  s1=1;
  else
  s1=0;
  if(digitalRead(lineFollowSensor1)==HIGH)
  s2=1;
  else
  s2=0;
  if(digitalRead(lineFollowSensor2)==HIGH)
  s3=1;
  else
  s3=0;
  if(digitalRead(lineFollowSensor3)==HIGH)
  s4=1;
  else
  s4=0;
  if(digitalRead(lineFollowSensor4)==HIGH)
  s5=1;
  else
  s5=0;
  if(digitalRead(lineFollowSensor5)==HIGH)
  s6=1;
  else
  s6=0;
  if(digitalRead(lineFollowSensor6)==HIGH)
  s7=1;
  else
  s7=0;  
}

void pid_work()
{
    right_speedd=right_speed+PIDvalue;
    if(right_speedd>255)
    right_speedd=255;
    left_speedd=left_speed-PIDvalue;
    if(left_speedd>255)
    left_speedd=255;
    bot(left_speedd,right_speedd);
}
///////////////////////////////////////////////////////////
void left_move()
{
  bot(-200,-200);
  delay(60);
  bot(0,0);
  delay(10);
 bot(-160,160);  //CCW  CW//hard hard left//turn start
 delay(170);//upto 45 degree
 s4=0;
 while(s4==0)
 {
   bot(-130,130);
   read_sensor();
 }
 bot(140,-140);
 delay(70);
 bot(0,0);
 delay(10);
// normal_line();
}
/////////////////////////////////////////////////////////

void left_move_short()
{
  bot(-200,-200);
  delay(60);
  bot(0,0);
  delay(10);
 bot(-160,160);  //CCW  CW//hard hard left//turn start
 delay(10);//upto 45 degree
 s4=0;
 while(s4==0)
 {
   bot(-130,130);
   read_sensor();
 }
 bot(140,-140);
 delay(70);
 bot(0,0);
 delay(10);

}
/////////////////////////////////////////////////////////
void right_move()
{
  bot(-200,-200);
  delay(60);
  bot(0,0);
  delay(10);
 bot(160,-160);  //CW   CCW//hard hard right//turn start
 delay(230); 
 s4=0;
 while(s4==0)
 {
  // BLUE_HI;
   bot(140,-140);  // CW    CCW//hard hard right
   read_sensor();
 }
 bot(-140,140);
 delay(60);
 bot(0,0);
 delay(10);
// normal_line_short();
}
void right_move_short()
{
  bot(-200,-200);
  delay(60);
  bot(0,0);
  delay(10);
 bot(160,-160);  //CW   CCW//hard hard right//turn start
 delay(10); 
 s4=0;
 while(s4==0)
 {
  // BLUE_HI;
   bot(140,-140);  // CW    CCW//hard hard right
   read_sensor();
 }
 bot(-140,140);
 delay(60);
 bot(0,0);
 delay(10);
// normal_line();
}

void back_turn()
{
  bot(-200,-200);
  delay(60);
  bot(0,0);
  delay(10);
 bot(160,-160);  //CW   CCW//hard hard right//turn start
 delay(10); 
 s5=0;
 while(s5==0)
 {
  // BLUE_HI;
   bot(160,-160);  // CW    CCW//hard hard right
   read_sensor();
 }
 bot(-140,140);
 delay(40);
 bot(0,0);
 delay(10);
// normal_line();
}
//===========================================================================//
void read_all_sensor()
{
  if(digitalRead(left_back)==HIGH)
  b1=1;
  else
  b1=0;
  if(digitalRead(right_back)==HIGH)
  b2=1;
  else
  b2=0;
  if(digitalRead(lineFollowSensor0)==HIGH)
  s1=1;
  else
  s1=0;
  if(digitalRead(lineFollowSensor1)==HIGH)
  s2=1;
  else
  s2=0;
  if(digitalRead(lineFollowSensor2)==HIGH)
  s3=1;
  else
  s3=0;
  if(digitalRead(lineFollowSensor3)==HIGH)
  s4=1;
  else
  s4=0;
  if(digitalRead(lineFollowSensor4)==HIGH)
  s5=1;
  else
  s5=0;
  if(digitalRead(lineFollowSensor5)==HIGH)
  s6=1;
  else
  s6=0;
  if(digitalRead(lineFollowSensor6)==HIGH)
  s7=1;
  else
  s7=0;
}
void check_mode()
{
  read_sensor();
  
  if((s1+s2+s3+s4+s5+s6+s7)==0)  
  {
    mode = STOPPED; 
    error = 0;
  }
  else if((s1+s2+s3+s4)>=3) 
  {
    mode = LEFT_FOUND; 
    error = 0;
  }
  else if((s4+s5+s6+s7)>=3) 
  {
    mode = RIGHT_FOUND; 
    error = 0;
  }
  /////////////////////////////////////////////////////////////////////////////////////////
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==1))  
  {
    mode = FOLLOWING_LINE; 
    error = -6;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==1)&&(s7==1))  
  {
    mode = FOLLOWING_LINE; 
    error = -5;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==1)&&(s7==0))  
  {
    mode = FOLLOWING_LINE; 
    error = -4;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==1)&&(s6==1)&&(s7==0))  
  {
    mode = FOLLOWING_LINE; 
    error = -3;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==1)&&(s6==0)&&(s7==0))  
  {
    mode = FOLLOWING_LINE; 
    error = -2;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==1)&&(s5==1)&&(s6==0)&&(s7==0))  
  {
    mode = FOLLOWING_LINE; 
    error = -1;
  }
  else if((s1==0)&&(s2==0)&&(s3==0)&&(s4==1)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 0;
  }
  else if((s1==0)&&(s2==0)&&(s3==1)&&(s4==1)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 1;
  }
  else if((s1==0)&&(s2==0)&&(s3==1)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 2;
  }
  else if((s1==0)&&(s2==1)&&(s3==1)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 3;
  }
  else if((s1==0)&&(s2==1)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 4;
  }
  else if((s1==1)&&(s2==1)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 5;
  }
  else if((s1==1)&&(s2==0)&&(s3==0)&&(s4==0)&&(s5==0)&&(s6==0)&&(s7==0)) 
  {
    mode = FOLLOWING_LINE; 
    error = 6;
  }  
}


void bot(int left, int right)// left motor value// right motor value
{
  if(left==0)
  {
    left_motor(0,left);
  }
  else if(left>0)
  {
    left_motor(1,left);
  }
  else
  {
    left_motor(2,-left);
  }
  /////////////////////////////////////
  if(right==0)
  {
    right_motor(0,0);
  }
  else if(right>0)
  {
    right_motor(1,right);
  }
  else
  {
    right_motor(2,-right);
  }
  
}
//////////////////////////////////////////////////////////////////
void left_motor(int dir, int spd)// sub function for left motor operation
{
  if(dir==1)// for forward
  {
    digitalWrite(n1,HIGH);
    digitalWrite(n2,LOW);
  }
  else if(dir==2)// for reverse
  {
    digitalWrite(n2,HIGH);
    digitalWrite(n1,LOW);
  }
  else// FOR STOP
  {
    digitalWrite(n2,LOW);
    digitalWrite(n1,LOW);
  }
  analogWrite(e1,spd);
}

void right_motor(int dir, int spd)// function for right motor operation
{
  if(dir==1)//for forward
  {
    digitalWrite(n3,HIGH);
    digitalWrite(n4,LOW);
  }
  else if(dir==2)// for reverse
  {
    digitalWrite(n4,HIGH);
    digitalWrite(n3,LOW);
  }
  else
  {
    digitalWrite(n3,LOW);
    digitalWrite(n4,LOW);
  }
  analogWrite(e2,spd);
}

////////////////////////////////////////////////////////
void wall_follow()
{
  BUZ_HI;
  bot(120,120);
  delay(200);
  BUZ_LOW;
  bot(-120,-120);
  delay(30);
  bot(0,0);
  delay(100);
  int wall=1;
  while(wall==1)
  {
    read_sonar(); 
    if(sr==0 && su==0 && sl==1)// may be 'C' point
    {
      bot(120,120);
      delay(100);
      read_sonar();
      if(sr==0 && su==0 && sl==1)
      {
       cave_follow();
       wall=0;
      }
    }
    else if((sr==1) && (su==1))// may be left
    {
      bot(120,120);
      delay(50);
      bot(-120,-120);
      delay(40);
      bot(0,0);
      read_sonar();
      if((sr==1) && (su==1))
      {
        wall_left_move();
      }
      else
      {
        wall_right_normal_line();
      }
    }
    //=========================================================================
    else if((sr==0) && (su==0))// may be right
    {
      bot(120,120);
      delay(50);
      bot(-120,-120);
      delay(30);
      bot(0,0);
      read_sonar();
      if((sr==0) && (su==0))
      {
          bot(120,120);
          delay(300);
          bot(-120,-120);
          delay(30);
          bot(0,0);
          wall_right_move();
      }
      else
      {
        wall_right_normal_line();
      }
    }
    else// follow wall
    {
      wall_right_normal_line();
    }
  }
}
//======================================================
void cave_follow()
{
  BUZ_HI;
  bot(120,120);
  delay(200);
  BUZ_LOW;
  bot(-120,-120);
  delay(30);
  bot(0,0);
  delay(100);
  int wal=1;
  while(wal==1)
  {
    read_sonar(); 
    if((sl==1) && (su==1))// may be right
    {
      bot(120,120);
      delay(50);
      bot(-120,-120);
      delay(40);
      bot(0,0);
      read_sonar();
      if((sl==1) && (su==1))
      {
        left_wall_right_move();
      }
      else
      {
        wall_left_normal_line();
      }
    }
    //=========================================================================
    else if((sl==0) && (su==0))// may be left
    {
      bot(120,120);
      delay(50);
      bot(-120,-120);
      delay(30);
      bot(0,0);
      read_sensor();
      if((s1+s2+s3+s4+s5+s6+s7)>=1)
       {
            bot(120,120);
            delay(40);
            read_sensor();
            if((s1+s2+s3+s4+s5+s6+s7)>=1)
            {
              wal=0;
            }
            else
            {
              bot(120,120);
              delay(300);
              bot(-120,-120);
              delay(30);
              bot(0,0);
              left_wall_left_move();
            }
       }
       else
       {
            read_sonar();
            if((sl==0) && (su==0))
            {
                bot(120,120);
                delay(300);
                bot(-120,-120);
                delay(30);
                bot(0,0);
                left_wall_left_move();  
            }
            else
            {
              wall_left_normal_line();
            }
          }
       }
    else// follow wall
    {
      wall_left_normal_line();
    }
  }
}
//======================================================

void wall_left_normal_line()
{
  read_sonar();
  l=left_sonar_read();
  if(l>0 && l<=5)
  {
    bot(100,40);
  }
  else if(l>5 && l<=7)
  {
    bot(100,80);
  }
  else if(l>7 && l<=10)
  {
    bot(100,100);
  }
  else if(l>10 && l<=12)
  {
    bot(100,120);
  }
  else if(l>12 && l<=15)
  {
    bot(80,120);
  }
  else if(l>15 && l<=17)
  {
    bot(60,120);
  }
  else if(l>17 && l<=20)
  {
    bot(40,120);
  }
  else if(l>20 && l<=23)
  {
    bot(20,120);
  }
  else if(l>23 && l<=25)
  {
    bot(10,120);
  }
  else if(l>25)
  {
    bot(0,120);
  }
  
}
//======================================================
void wall_right_normal_line()
{
  read_sonar();
  r=right_sonar_read();
  if(r>0 && r<=5)
  {
    bot(50,120);
  }
  else if(r>5 && r<=7)
  {
    bot(90,120);
  }
  else if(r>7 && r<=10)
  {
    bot(100,120);
  }
  else if(r>10 && r<=12)
  {
    bot(120,120);
  }
  else if(r>12 && r<=15)
  {
    bot(120,100);
  }
  else if(r>15 && r<=17)
  {
    bot(120,80);
  }
  else if(r>17 && r<=20)
  {
    bot(120,60);
  }
  else if(r>20 && r<=23)
  {
    bot(120,40);
  }
  else if(r>23 && r<=25)
  {
    bot(120,20);
  }
  else if(r>25)
  {
    bot(120,10);
  }
  
}
//=======================================================
void wall_right_move()
{
  bot(-120,-120);
  delay(200);
  bot(120,120);
  delay(30);
  bot(0,0);
  delay(100);
  bot(160,0);
  delay(520);
  bot(-120,0);
  delay(30);
  bot(120,120);
  delay(300);

  bot(-130,0);
  delay(40);
  bot(0,0);
  delay(100);
}

void wall_left_move()
{
  bot(-120,-120);
  delay(30);
  bot(0,0);
  delay(100);
  bot(-120,120);
  delay(380);
  sr=0;

  bot(120,-120);
  delay(40);
  bot(0,0);
  delay(100);
}
//=====================================================
void left_wall_left_move()
{

  bot(120,120);
  delay(50);
  bot(0,0);
  delay(100);
  bot(0,160);
  delay(500);
  bot(0,-120);
  delay(30);
  bot(120,120);
  delay(300);

  bot(0,-130);
  delay(40);
  bot(0,0);
  delay(100);
}

void left_wall_right_move()
{
  bot(-120,-120);
  delay(30);
  bot(0,0);
  delay(100);
  bot(120,-120);
  delay(380);
  sr=0;

  bot(-120,120);
  delay(40);
  bot(0,0);
  delay(100);
}
//====================================================
void read_sonar()
{
  length_lft=left_sonar_read();
  length_rght=right_sonar_read();
  length_frd=forward_sonar_read();
  if(length_frd!=0 && length_frd<=11)
  {
    su=1;
  }
  else
  {
    su=0;
  }
  if(length_rght!=0 && length_rght<=28)
  {
    sr=1;
  }
  else
  {
    sr=0;
  }
  
  if(length_lft!=0 && length_lft<=28)
  {
    sl=1;
  }
  else
  {
    sl=0;
  }
  
}
//=====================================================//
float forward_sonar_read()
{
  digitalWrite(trigpin_frd, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin_frd, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin_frd, LOW);
  duration_frd = pulseIn(echopin_frd, HIGH,1800);
  cm_frd = (duration_frd/2) / 29.1;
  return cm_frd;
}

/////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////

float left_sonar_read()
{
  digitalWrite(trigpin_lft, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin_lft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin_lft, LOW);
  duration_lft = pulseIn(echopin_lft, HIGH,1800);
  cm_lft= (duration_lft/2) / 29.1;
  return cm_lft;
}
////////////////////////////////////////////////////////////////////

float right_sonar_read()
{
  digitalWrite(trigpin_rght, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin_rght, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin_rght, LOW);
  duration_rght = pulseIn(echopin_rght, HIGH,1500);
  cm_rght= (duration_rght/2) / 29.1;
  return cm_rght;
}
////////////////////////////////////////////////////////////////////
void check()
{
  BUZ_LOW;
  bot(-100,-100);
  delay(40);
  bot(0,0);
  while(1);
}
void end_found()
{
  int end_fnd=1;
  while(end_fnd==1)
  {
    BUZ_HI;
    digitalWrite(led,HIGH);
    delay(150);
    BUZ_LOW;
    digitalWrite(led,LOW);
    delay(150);
  }
}
