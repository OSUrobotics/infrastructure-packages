//Last Updated: Brody Sears - 7/22/2020

#include <AccelStepper.h>
#include <Wire.h>
int I2C_SLAVE2 = 14;
/***************************************************************************************************************************************************************/

//Variable Declarations
volatile byte received = 0;
volatile byte initial_received = 0;
int n = 10;
float object_array[6];
int swap_array[2];
bool data_transfer = false;
float transferred_data = 0;
int swap_data= 0;
int current_location = 0;
bool swap_data_bool = false;
int current_swap_location = 0;

//Stepper Pin Declaration
int yPUL = 8;
int yDIR = 9;
int xPUL = 13;
int xDIR = 12;
int rPUL = 7;
int rDIR = 6;
bool run_var = false;
bool run_var2 = false;

bool end_bool = false;

//Magnet pin Declaration
int magnetIn1 = 28;
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
int limitrotationalpin = 35;


//Stepper Declarations
AccelStepper Vertical_Stepper(1,  yPUL, yDIR);
AccelStepper Horizontal_Stepper(1, xPUL, xDIR);
AccelStepper Rotational_Stepper(1, rPUL, rDIR); 


/***************************************************************************************************************************************************************/


void setup() {

    //initialize serial moniter
  Serial.begin(57600);
  Wire.begin(I2C_SLAVE2);
  Wire.setClock( 100000L);

   pinMode(magnetEN, OUTPUT);
  pinMode(magnetIn1, OUTPUT);
  pinMode(magnetIn2, OUTPUT); 

  //initialize limit input pins
  pinMode(limitverticalpin, INPUT);
  pinMode(limithorizontalpin, INPUT);
  pinMode(limitrotationalpin, INPUT);

  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  Horizontal_Stepper.setMaxSpeed(500);
  Horizontal_Stepper.setAcceleration(80);
  Vertical_Stepper.setMaxSpeed(1000);
  Vertical_Stepper.setAcceleration(80);
  Rotational_Stepper.setMaxSpeed(300);
  Rotational_Stepper.setAcceleration(80);
}

/***************************************************************************************************************************************************************/
void move_stepper(AccelStepper stepper,int position_val){
  stepper.moveTo(position_val);
 while(stepper.distanceToGo()){
  stepper.moveTo(position_val);
  stepper.run();
 }
}
void move_To_Limit(AccelStepper stepper,int limitpin){
  stepper.setSpeed(-500);
  while(digitalRead(limitpin) == 0)
    stepper.runSpeed();
  stepper.setCurrentPosition(0);
}
/***************************************************************************************************************************************************************/
void reach_Limit_Vertical(){
  Vertical_Stepper.setCurrentPosition(0);
  move_stepper(Vertical_Stepper,1000);
  move_To_Limit(Vertical_Stepper,limitverticalpin);
 move_stepper(Vertical_Stepper,8000);
 }
void reach_Limit_Horizontal(){
  move_To_Limit(Horizontal_Stepper,limithorizontalpin);
  move_stepper(Horizontal_Stepper,200);
 }
void reach_Limit_Rotational(){
  move_To_Limit(Rotational_Stepper,limitverticalpin);
}
/***************************************************************************************************************************************************************/
void turn_On_Magnet_North(){
digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, HIGH);
digitalWrite(magnetIn2, LOW);
}
void turn_On_Magnet_South(){
  digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, LOW);
digitalWrite(magnetIn2, HIGH);
}
void turn_Off_Magnet(){
  digitalWrite(magnetEN,HIGH);
digitalWrite(magnetIn1, LOW);
digitalWrite(magnetIn2, LOW);
}
/***************************************************************************************************************************************************************/
void grab_Object(){
reach_Limit_Vertical();
reach_Limit_Horizontal();
reach_Limit_Rotational();
move_stepper(Horizontal_Stepper,2400);
turn_On_Magnet_North();
move_stepper(Vertical_Stepper,6150);
move_stepper(Vertical_Stepper,16000);
}
/***************************************************************************************************************************************************************/
void swap_Object(){
//Dropping off Object
move_stepper(Horizontal_Stepper,200);
move_stepper(Rotational_Stepper,4675);
move_stepper(Horizontal_Stepper,0);
move_stepper(Vertical_Stepper,10950);
turn_On_Magnet_South();
delay(500);

//Grabbing new Object
move_stepper(Vertical_Stepper,15000);
move_stepper(Horizontal_Stepper,1275);
turn_On_Magnet_North();
move_stepper(Vertical_Stepper,13100);
delay(500);

//Moving to place New Object
move_stepper(Vertical_Stepper,16000);
move_stepper(Horizontal_Stepper,200);
move_stepper(Rotational_Stepper,0);
move_stepper(Horizontal_Stepper,2400);
move_stepper(Vertical_Stepper,8050);
turn_On_Magnet_South();
delay(500);

//Returning arm to rest position
move_stepper(Vertical_Stepper,10000);
turn_Off_Magnet();
move_stepper(Horizontal_Stepper,200);
move_stepper(Rotational_Stepper,2750);
move_stepper(Vertical_Stepper,0);
delay(250);
}


/***************************************************************************************************************************************************************/

void loop() {
  if(data_transfer){
  object_array[current_location] = transferred_data;
  }
  if(swap_data_bool){
  swap_array[current_swap_location] = swap_data;
  }
  for(int i = 0; i < 6; i++)
{
  Serial.print(object_array[i]);
}
Serial.println(" ");
  if(run_var){
    grab_Object();
    delay(50);
    run_var = false;
    end_bool = true;
    //Serial.println("Finished grab object loop");
  }
  if(run_var2){  
    //Serial.println("Entered run_var2") ;
    end_bool = false;
    swap_Object();
    run_var2 = false;
    end_bool = true;
  }
  if (initial_received > 10 && initial_received < 100)
    transferred_data = float(initial_received)/10.0;
  if (initial_received >= 100 && initial_received < 107)
    swap_data = initial_received-100;
  if (received != 255)
    initial_received = received;

}
/***************************************************************************************************************************************************************/
void requestEvents(){
  switch(initial_received){
    case 2:
      Wire.write(0);
      run_var = true;
      initial_received = 0;
      break;
      
    case 3:
      if (end_bool)
        Wire.write(3);
      else
        Wire.write(0);
      break;
      
    case 4:
      Wire.write(0);
      run_var2 = true;
      initial_received = 0;
      break;
      
    case 5:
      if(end_bool)
        Wire.write(5);
      else
        Wire.write(0);
    break;
    case 6:
      data_transfer = true;
      break;
    case 7:
      current_location++;
      data_transfer = false;
      break;
      case 8: 
      data_transfer = false;
      break;
      case 9:
      transferred_data = 0;
      break;
      case 10:
      swap_data_bool = true;
      case 11:
      current_swap_location++;
      swap_data_bool = false;
      break;
    default:
      break;
  }
}
/***************************************************************************************************************************************************************/

void receiveEvents(int numBytes){
    Wire.read();
    received = Wire.read();
}
