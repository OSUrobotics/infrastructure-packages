#include <Wire.h>
#define I2C_SLAVE 15
/*********************************************************************************/
volatile byte received = 0;
volatile byte initial_received = 0;

int steps = 0;
int current_steps = 0;
byte stepbytes[2];

bool counting = false;
bool swap = true;

const int hall_effect_pin = 10;
const int cone_button_pin = 6;
const int limit_switch_pin = 11;
const int encoder_pin = 3;
bool varl = 0;
bool varb = 0;
bool varh = 0;

/*********************************************************************************/
void setup() {
  Serial.begin(57600);

  pinMode(encoder_pin, INPUT);
  pinMode(limit_switch_pin, INPUT); //wasn't here before?
  pinMode(cone_button_pin, INPUT);
  pinMode(hall_effect_pin, INPUT);
  
  Wire.begin(I2C_SLAVE);
  delay(1000);
  //Wire.setClock( 100000L);
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

attachInterrupt(digitalPinToInterrupt(encoder_pin), step_counter, CHANGE); //is this to record step count of turn motor?
}
/*********************************************************************************/
void loop() {
  if (received != 255)
    initial_received = received;
    
  /*
  varl = digitalRead(limit_switch_pin);
  varb = digitalRead(cone_button_pin);
  varh = digitalRead(hall_effect_pin);
  */
  //Serial.println("ls: " + String(varl) + " -- he: " + String(varh) + " --- cb: " + String(varb));
  //stepbytes[0]=lowByte(current_steps);
  //stepbytes[1]= highByte(current_steps);
  
  if (counting == false){
      current_steps = 0;
  }
}
/*********************************************************************************/
void step_counter(){
  if (counting == true){
    current_steps++;
  }
}
/*********************************************************************************/
void requestEvents(){
  //Serial.println("Requested an event");
  switch(initial_received){
    case 2:
      counting=true;
      break;
    case 3:
      //Wire.write(varl);
      Wire.write(digitalRead(limit_switch_pin));
      break;
    case 4:
      //Wire.write(varb);
      Wire.write(digitalRead(limit_switch_pin));
      break;
    case 5: 
      //Wire.write(varh);
      Wire.write(digitalRead(hall_effect_pin));
      break;
    case 6:
      if(swap){
        Wire.write(highByte(current_steps));
        swap = false;
      }
      else{
        Wire.write(lowByte(current_steps));
        swap = true;
      }
      break;
    case 7: 
      counting = false;
    case 8:
      swap = true;
      break;
    default:
      break;
  }
}
/*********************************************************************************/
  void receiveEvents(int numBytes){
    Wire.read();
    received = Wire.read();
}
/*********************************************************************************/
