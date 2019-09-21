// Definitive version of FiveBarQuad - 21/09/2019
// This program uses my Inverse Kinematics function to move the FiveBarQuad-3 robot
#include <Servo.h>
const int nbSrv=8;                                          // Number of servos used
Servo Srv[nbSrv];                                           // Servos table
int SrvOn[nbSrv]={1,1,1,1,1,1,1,1};                         // Switch servos table 1=On 0=Off
//int SrvOn[nbSrv]={0,0,0,0,0,0,0,0};                         // Switch servos table 1=On 0=Off

int i, Speed = 4;                                              // to decrease the speed, increase the value

//Rectangle movement step 5 mm with 15 to 25 gap - 21/09/2019 - 
int FRx[]= { 20, 15, 10,  5,  0, -5,-10,-15,-20,    -20,-15,-10, -5,  0,  5, 10, 15, 20};
int BRx[]= {-20,-15,-10, -5,  0,  5, 10, 15, 20,     20, 15, 10,  5,  0, -5,-10,-15,-20};
int FLx[]= { 20, 15, 10,  5,  0, -5,-10,-15,-20,    -20,-15,-10, -5,  0,  5, 10, 15, 20};
int BLx[]= {-20,-15,-10, -5,  0,  5, 10, 15, 20,     20, 15, 10,  5,  0, -5,-10,-15,-20};

int FRy[]= { 15, 15, 15, 15, 15, 15, 15, 15, 15,     25, 25, 25, 25, 25, 25, 25, 25, 25};
int BRy[]= { 25, 25, 25, 25, 25, 25, 25, 25, 25,     15, 15, 15, 15, 15, 15, 15, 15, 15};
int FLy[]= { 25, 25, 25, 25, 25, 25, 25, 25, 25,     15, 15, 15, 15, 15, 15, 15, 15, 15};
int BLy[]= { 15, 15, 15, 15, 15, 15, 15, 15, 15,     25, 25, 25, 25, 25, 25, 25, 25, 25};

int lgTab = sizeof(FRx)/sizeof(int);

int Trig_F=A0, Trig_L=A2, Trig_R=A4;                        //US sensor HC-SR04 connectors and variables
int Echo_F=A1, Echo_L=A3, Echo_R=A5;                        //US sensor HC-SR04 connectors and variables
float distance_F=400, distance_L=400, distance_R=400;       //US sensor HC-SR04 connectors and variables

void  setup() {
  Serial.begin(9600);
  pinMode(Trig_F, OUTPUT); pinMode(Echo_F, INPUT);          // Ultrasonic sensor HC-SR04 Front
  pinMode(Trig_L, OUTPUT); pinMode(Echo_L, INPUT);          // Ultrasonic sensor HC-SR04 Left
  pinMode(Trig_R, OUTPUT); pinMode(Echo_R, INPUT);          // Ultrasonic sensor HC-SR04 Right

  pinMode(0,INPUT_PULLUP);                                  // start button attachment
  
  Srv[0].attach(2);  Srv[1].attach(3);                      // Front Right leg
  Srv[2].attach(6);  Srv[3].attach(7);                      // Back Right leg
  Srv[4].attach(4);  Srv[5].attach(5);                      // Front left leg
  Srv[6].attach(8);  Srv[7].attach(9);                      // Back left leg
    
  Srv[0].write(90);  Srv[1].write(90);                      // set servo to neutral point
  Srv[2].write(90);  Srv[3].write(90);                      // set servo to neutral point
  Srv[4].write(90);  Srv[5].write(90);                      // set servo to neutral point
  Srv[6].write(90);  Srv[7].write(90);                      // set servo to neutral point
  
  while( digitalRead(0) );                                  // wait for start button pressed
//  Forward();
}

void  loop(){
  US_detect();

  if (distance_F <= 60){
    if (distance_L < distance_R)  {for (i=0;i<1;i++){Backward();}for (i=0;i<4;i++){TurnRight();}}
    else                          {for (i=0;i<1;i++){Backward();}for (i=0;i<4;i++){TurnLeft();}}
  }
  
  if (distance_L <= 30) {if (distance_L < distance_R){for (i=0;i<1;i++){TurnRight();}}}
  if (distance_R <= 30) {if (distance_R < distance_L){for (i=0;i<1;i++){TurnLeft();}}}

  Forward();
}

void Forward(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the target points from the beginning to the end
    InverseKinematics(FRx[i],FRy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[i],BRy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(FLx[i],FLy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(BLx[i],BLy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  }  
}

void Backward(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the target points from the end to the beginning
    InverseKinematics(FRx[lgTab-1-i],FRy[lgTab-1-i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[lgTab-1-i],BRy[lgTab-1-i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(FLx[lgTab-1-i],FLy[lgTab-1-i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(BLx[lgTab-1-i],BLy[lgTab-1-i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  } 
}

void TurnLeft(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the right legs to turn left
    InverseKinematics(FRx[i],FRy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BRx[i],BRy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(BRx[i],BRy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(FRx[i],FRy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]);
  }  
}

void TurnRight(){
  for(int i=0;i<lgTab;i++){                                     // Browse the tables of the left legs to turn right
    InverseKinematics(FLx[i],FLy[i],Srv[0],Srv[1],SrvOn[0],SrvOn[1]);
    InverseKinematics(BLx[i],BLy[i],Srv[2],Srv[3],SrvOn[2],SrvOn[3]);
    InverseKinematics(BLx[i],BLy[i],Srv[4],Srv[5],SrvOn[4],SrvOn[5]);
    InverseKinematics(FLx[i],FLy[i],Srv[6],Srv[7],SrvOn[6],SrvOn[7]); 
  } 
}

void InverseKinematics(int Px, int Py, Servo srvL, Servo srvR, int srvLon, int srvRon){       // below, 12 lines to calculate the angle of the servos
  int g=32;                                                     //  1- Interval between the two servos
  float A1x=-16, A1y=180, A2x=A1x+g, A2y=A1y;                   //  2- Values of the servos positions
  float a1=152, c1=32, a2=a1, c2=c1;                            //  3- Values of the length of the leg's bars
  float d1=abs(A1y-Py), d2=abs(A2y-Py);                         //  4- values of the d sides of the virtual right-angle triangles
  float e1=abs(A1x-Px), e2=abs(A2x-Px);                         //  5- value of e sides of the virtual right-angle triangles
  float b1=sqrt(sq(d1)+sq(e1)), b2=sqrt(sq(e2)+sq(d2));         //  6- value of b sides (hypotenuse) of the virtual right-angle triangles 
  if ( b1>(a1+c1) || b2>(a1+c1) ){                              //    test if the target point is reachable
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" is too far. Impossible to reach !!!");Serial.print("\n");
    return;
  }
  float A_1=(acos((sq(b1)+sq(c1)-sq(a1))/(2*b1*c1)))*57.296;    //  7- Value of the A angle of the left triangle with cosine law. Radian to degree convertion
  float A_2=(acos((sq(b2)+sq(c2)-sq(a2))/(2*b2*c2)))*57.296;    //  8- Value of the A angle of the right triangle with cosine law. Radian to degree convertion
  float D1=(acos((sq(g)+sq(b1)-sq(b2))/(2*b1*g)))*57.296;       //  9- Value of the D left angle of the center triangle with cosine law. Radian to degree convertion
  float D2=(acos((sq(g)+sq(b2)-sq(b1))/(2*b2*g)))*57.296;       // 10- Value of the D right angle of the center triangle with cosine law. Radian to degree convertion

  int S1=round(180-A_1-D1);                                     // 11- Angle value of the left servo
  int S2=round(A_2+D2);                                         // 12- Angle value of the right servo
  if ( S1<5 || S2>175 ){                                        // if target angle is too short or too large, it is reject
    Serial.print("\n\t Target point Px=");Serial.print(Px);Serial.print(" Py=");Serial.print(Py);Serial.print(" angle is impossible to reach !!!");Serial.print("\n");
    return;
  }
  if (srvLon) srvL.write(S1);                                   // set target Left servo position if servo switch is On
  if (srvRon) srvR.write(S2);                                   // set target Right servo position if servo switch is On
  delay(Speed);
/*
  // DEBUG 1
  Serial.print(" Px=");Serial.print(Px);Serial.print("\tPy=");Serial.print(Py);
  Serial.print("\t\tS1=");Serial.print(S1);Serial.print("°\tS2=");Serial.print(S2);Serial.print("°\n");
/*
/* 
  // DEBUG 2
  Serial.print("\n\t Position to reach : Px=");Serial.print(Px);Serial.print("  Py=");Serial.print(Py);Serial.print("\n\n");
  Serial.print("\t Left servo \t\t Right servo\n\n");  
  Serial.print("\t A1x=");Serial.print(A1x);Serial.print("\t\t A2x=");Serial.print(A2x);Serial.print("\n");
  Serial.print("\t A1y=");Serial.print(A1y);Serial.print("\t\t A2y=");Serial.print(A2y);Serial.print("\n");
  Serial.print("\t a1=");Serial.print(a1);Serial.print("\t\t a2=");Serial.print(a2);Serial.print("\n");
  Serial.print("\t c1=");Serial.print(c1);Serial.print("\t\t c2=");Serial.print(c2);Serial.print("\n");  
  Serial.print("\t d1=");Serial.print(d1);Serial.print("\t\t d2=");Serial.print(d2);Serial.print("\n");
  Serial.print("\t e1=");Serial.print(e1);Serial.print("\t\t e2=");Serial.print(e2);Serial.print("\n");
  Serial.print("\t b1=");Serial.print(b1);Serial.print("\t\t b2=");Serial.print(b2);Serial.print("\n");
  Serial.print("\t A_1=");Serial.print(A_1);Serial.print("°\t\t A_2=");Serial.print(A_2);Serial.print("°\n");
  Serial.print("\t D1=");Serial.print(D1);Serial.print("°\t\t D2=");Serial.print(D2);Serial.print("°\n\n");
  Serial.print("\t Result of calculations, angles of the servos\n\n");
  Serial.print("\t S1=");Serial.print(S1);Serial.print("°\t\t\t S2=");Serial.print(S2);Serial.print("°\n\n\n");
*/
}

void US_detect(){                                               // US sensor HC-SR04 function sensor
  digitalWrite(Trig_F,LOW);   delayMicroseconds(2);
  digitalWrite(Trig_F,HIGH);  delayMicroseconds(10);
  digitalWrite(Trig_F,LOW);   distance_F= pulseIn(Echo_F, HIGH) / 58.0;
//  Serial.print("\tDistance US_F\t");Serial.print(distance_F);Serial.print(" cm ");                
  digitalWrite(Trig_L, LOW);  delayMicroseconds(2);
  digitalWrite(Trig_L, HIGH); delayMicroseconds(10);
  digitalWrite(Trig_L, LOW);  distance_L= pulseIn(Echo_L, HIGH) / 58.0;
//  Serial.print("\tDistance US_L\t");Serial.print(distance_L);Serial.print(" cm ");                
  digitalWrite(Trig_R, LOW);  delayMicroseconds(2);
  digitalWrite(Trig_R, HIGH); delayMicroseconds(10);
  digitalWrite(Trig_R, LOW);  distance_R= pulseIn(Echo_R, HIGH) / 58.0;
//  Serial.print("\tDistance US_R\t");Serial.print(distance_R);Serial.print(" cm ");
//  Serial.print("\n");
}
