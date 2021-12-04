#include <Wire.h>
#define SLAVE 15
#define hall_effect_pin 10
#define cone_button_pin 6
#define limit_switch_pin 11
#define encoder_pin 3
/*********************************************************************************/
volatile byte received = 0;
volatile byte initial_received = 0;

int steps = 0;
int current_steps = 0;
byte data[2];

bool counting = false;
bool swap = true;

bool varl = 0;
bool varb = 0;
bool varh = 0;

/*********************************************************************************/
void setup() {
  Serial.begin(57600);

  pinMode(encoder_pin, INPUT);
  pinMode(limit_switch_pin, INPUT);
  pinMode(cone_button_pin, INPUT);
  pinMode(hall_effect_pin, INPUT);
  
  Wire.begin(SLAVE);
  delay(1000);
  Wire.setClock(100000);
  Wire.onRequest(requestEventHandler);
  Wire.onReceive(receiveEventHandler);

  attachInterrupt(digitalPinToInterrupt(encoder_pin), stepCounter, CHANGE);
}
/*********************************************************************************/
void loop() {
//  if (received != 255)
//    initial_received = received;
    
  
//  varl = digitalRead(limit_switch_pin);
//  varb = digitalRead(cone_button_pin);
//  varh = digitalRead(hall_effect_pin);
  
  //Serial.println("ls: " + String(varl) + " -- he: " + String(varh) + " --- cb: " + String(varb));
  //stepbytes[0]=lowByte(current_steps);
  //stepbytes[1]= highByte(current_steps);
  
//  if (counting == false){
//      current_steps = 0;
//  }
}
/*********************************************************************************/
void stepCounter(){
  if (counting == true){
    current_steps++;
    Serial.print(" --- ");
    Serial.println(current_steps);
  }
}
/*********************************************************************************/
void requestEventHandler(){
  //Serial.println("Requested an event");
  switch(received){
//    case 2:
//      counting=true;
//      break;
    case 3:
      data[0] = 0;
      data[1] = digitalRead(limit_switch_pin);
      Wire.write(data, 2);
      //Wire.write(digitalRead(limit_switch_pin)); //works for single byte read
      break;
    case 4:
      data[0] = 0;
      data[1] = digitalRead(cone_button_pin);
      Wire.write(data, 2);
      //Wire.write(digitalRead(cone_button_pin)); //works for single byte read
      break;
    case 5: 
      data[0] = 0;
      data[1] = digitalRead(hall_effect_pin);
      Wire.write(data, 2);
      // Wire.write(digitalRead(hall_effect_pin)); //works for single byte read
      break;
    case 6:
      data[0] = highByte(current_steps);
      data[1] = lowByte(current_steps);
      Wire.write(data, 2);
      break;
//    case 7: 
//      counting = false;
//      current_steps = 0;
    case 8:
      swap = true;
      break;
    default:
      break;
  }
}
/*********************************************************************************/
  void receiveEventHandler(int num_bytes){ // why being sent two data frames?
    //Wire.read();
    //Serial.println(num_bytes);
    received = Wire.read();
    if(received == 6){
      counting = true;  
    }
    else if(received == 7){
      counting = false;
      current_steps = 0;
    }
    Serial.println(received);
}
/*********************************************************************************/
