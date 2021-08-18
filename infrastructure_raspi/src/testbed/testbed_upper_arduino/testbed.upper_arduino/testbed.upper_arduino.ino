//Last Updated: Brody Sears - 7/22/2020

#include <AccelStepper.h>
#include <Servo.h>
#include <SPI.h>

class Object{
  float height;
  int level;
  int arc_position;
  bool cone_bool;

 public:

  void set_Cone_Bool(bool bools){cone_bool = bools;}
  void set_Height(float new_height){height = new_height;}
  void set_Level(int new_level){level = new_level;}
  void set_Arc_Position(int new_Arc_Position){arc_position = new_Arc_Position;}

  int get_Arc_Position(){return arc_position;}
  int get_Level(){return level;}
  float get_Height(){return height;}
  bool get_Cone_Bool(){return cone_bool;}

  Object(float new_height,int new_level, int new_arc_position){height = new_height; level= new_level; arc_position = new_arc_position; cone_bool = false;}
  Object(float new_height, int new_level, int new_arc_position, bool new_bool){height = new_height; level= new_level; arc_position = new_arc_position; cone_bool = new_bool;}
  ~Object(){} 
};
/********************************************************************/

//Variable Declarations
volatile byte received = 0;


//Stepper Pin Declaration
int yPUL = 8;
int yDIR = 9;
int xPUL = 13;
int xDIR = 12;
int rPUL = 7;
int rDIR = 6;

//Magnet pin Declaration
int magnetIn1 = 29;
int magnetIn2 = 29;
int magnetEN = 11;


//Height and misc variables
int vertical_level_1_position;
int vertical_level_2_position;
int vertical_level_3_position;
int bed_level;
int object_height_offset;
int inch_to_position_conversion;
int middle_horizontal_position;

//limit switch analog pins
int limitverticalpin = 37;
int limithorizontalpin = 43;
int limitrotationalpin = 41;


//Stepper Declarations
AccelStepper Vertical_Stepper(1,  yPUL, yDIR);
AccelStepper Horizontal_Stepper(1, xPUL, xDIR);
AccelStepper Rotational_Stepper(1, rPUL, rDIR); 


/********************************************************************/


void setup() {
    //initialize serial moniter
  Serial.begin(57600);

   pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

   pinMode(magnetEN, OUTPUT);
  pinMode(magnetIn1, OUTPUT);
  pinMode(magnetIn2, OUTPUT); 

  //initialize limit input pins
  pinMode(limitverticalpin, INPUT);
  pinMode(limithorizontalpin, INPUT);
  pinMode(limitrotationalpin, INPUT);

  Horizontal_Stepper.setMaxSpeed(300);
  Horizontal_Stepper.setAcceleration(80);
  Vertical_Stepper.setMaxSpeed(300);
  Vertical_Stepper.setAcceleration(80);
  Rotational_Stepper.setMaxSpeed(300);
  Rotational_Stepper.setAcceleration(80);
}

/********************************************************************/

void reach_Limit_Vertical(){
  Vertical_Stepper.setCurrentPosition(0);
    Vertical_Stepper.setMaxSpeed(1000);
  Serial.println("Started Primery Vertical Shift ");
  for (int i=0; i < 8000; i++){
   Vertical_Stepper.setSpeed(500);
  delay(1);
  Serial.println(i);
  Vertical_Stepper.runSpeed();
  }
  Serial.println("Finished Primery Vertical Shift ");

  Vertical_Stepper.setSpeed(-500);
  while(digitalRead(limitverticalpin) == 0){
  Vertical_Stepper.runSpeed();
   }
   Vertical_Stepper.setCurrentPosition(0);

while(Vertical_Stepper.currentPosition() != 8000){
   Vertical_Stepper.setSpeed(500);
   delay(1);
    Serial.println(Vertical_Stepper.currentPosition());
    Vertical_Stepper.runSpeed();  
  }
   
 }
/********************************************************************/

 
void reach_Limit_Horizontal(){
  Horizontal_Stepper.setSpeed(-500);
  while(digitalRead(limithorizontalpin)== 0){
  Horizontal_Stepper.runSpeed();
  }
    Serial.println("Finished contact with Limit Switch");
    Serial.println("Setting Position");

  Horizontal_Stepper.setCurrentPosition(0);
Serial.println("Moving to Offset");
  while (Horizontal_Stepper.currentPosition() != 450) {
  Horizontal_Stepper.setSpeed(500);
  delay(1);
  Serial.println(Horizontal_Stepper.currentPosition());
  Horizontal_Stepper.runSpeed();
  }
 }
  /********************************************************************/

void reach_Limit_Rotational(){
      Serial.println("Starting Rotation");
  Rotational_Stepper.setSpeed(-400);
  while(digitalRead(limitrotationalpin)==0){
  Rotational_Stepper.runSpeed();
  }
  Rotational_Stepper.setCurrentPosition(0);
}
/********************************************************************/
void go_To_Horizontal_Middle(){
  while (Horizontal_Stepper.currentPosition() != 2500) {
  Horizontal_Stepper.setSpeed(500);
  delay(1);
      Horizontal_Stepper.runSpeed();
  Serial.println(Horizontal_Stepper.currentPosition());}
}
/********************************************************************/
int object_Height_To_Position(float height){ return (height * inch_to_position_conversion);}

/********************************************************************/
void go_To_Object(Object current_object){
//int height = current_object.get_Height();
//Vertical_Stepper.moveTo(bed_level - object_Height_To_Position(height));
while(Vertical_Stepper.currentPosition() != 6100){
   Vertical_Stepper.setSpeed(-500);
   delay(1);
    Serial.println(Vertical_Stepper.currentPosition());
    Vertical_Stepper.runSpeed();  
  }
}
/********************************************************************/
void turn_On_Magnet_North(){
digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, LOW);
digitalWrite(magnetIn2, HIGH);
delay(2000);
}
/********************************************************************/
void go_To_Vertical_Level(Object current_object){
//int level = current_object.get_Level();
//if(level == 1){Vertical_Stepper.moveTo(vertical_level_1_position);}
//else if(level == 2){Vertical_Stepper.moveTo(vertical_level_2_position);}
//else {Vertical_Stepper.moveTo(vertical_level_3_position);}
//
while(Vertical_Stepper.currentPosition() != 9000){
   Vertical_Stepper.setSpeed(500);
   delay(1);
    Serial.println(Vertical_Stepper.currentPosition());
    Vertical_Stepper.runSpeed();  
  }
}

/********************************************************************/
void go_To_Arc(Object current_object){
  int arc_position = current_object.get_Arc_Position();
  Rotational_Stepper.moveTo(arc_position);
}

/********************************************************************/
void go_To_Level_Object_Height(Object current_object){
int level = current_object.get_Level();
float height = current_object.get_Height();
if(level==1){Vertical_Stepper.moveTo(vertical_level_1_position + object_height_offset - object_Height_To_Position(height));}
else if(level == 2){Vertical_Stepper.moveTo(vertical_level_2_position + object_height_offset - object_Height_To_Position(height));}
else {Vertical_Stepper.moveTo(vertical_level_3_position + object_height_offset - object_Height_To_Position(height));}


}
/********************************************************************/
void turn_On_Magnet_South(){
  digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, LOW);
digitalWrite(magnetIn2, HIGH);
}

/********************************************************************/
void raise_Offset(Object current_object){
  int level = current_object.get_Level();
  if(level == 1){Vertical_Stepper.moveTo(vertical_level_1_position);}
  else if (level == 2){Vertical_Stepper.moveTo(vertical_level_2_position);}
  else{Vertical_Stepper.moveTo(vertical_level_3_position);}
}
/********************************************************************/
void turn_Off_Magnet(){
  digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, LOW);
digitalWrite(magnetIn2, LOW);

}
/********************************************************************/

void grab_Object(){
Object current_object(4, 1, 1, true);
Serial.println("Attempting Vertical Limit ");
reach_Limit_Vertical();
Serial.println("Reached Vertical Limit ");

Serial.println("Attempting Horizontal Limit ");
reach_Limit_Horizontal();
Serial.println("Reached Horizontal Limit ");

Serial.println("Attempting Rotational Limit ");
reach_Limit_Rotational();
Serial.println("Finished Horizontal Limit ");

Serial.println("Aligning Arm to Center ");
go_To_Horizontal_Middle();
Serial.println("Going to Object");
go_To_Object(current_object);
Serial.println("Turning on Magnet");
turn_On_Magnet_North();
Serial.println("Raising object up");
go_To_Vertical_Level(current_object);

//Horizontal_Stepper.moveTo(0);
//go_To_Arc(current_object);
//go_To_Level_Object_Height(current_object);
//turn_On_Magnet_South();
//raise_Offset(current_object);
//turn_Off_Magnet();
}
/********************************************************************/

void loop() {
//Serial.println("Enter anything to start: ");
//    while(Serial.available() < 1){
//      delay(1);}
}
/********************************************************************/

ISR (SPI_STC_vect)
{
  received = SPDR;

  if (received == 1)
  {
    SPDR = 2;
  }
  else if (received == 2)
  {
    grab_Object();
    SPDR = 3;
  } 

  else if (received == 4)
  {
    //swap_Objects();
  }
}
