#include <SPI.h>

int encoder_pin = 2;
int steps = 0;
bool counting = false;
volatile byte received = 0;
byte Send_part1 = 0;
byte Send_part2 = 0;
int current_step = 0;
volatile byte current_step_part1 = 0;
volatile byte current_step_part2 = 0;

const int hall_effect_pin = 0;
bool hall_effect_val = HIGH;

const int cone_button_pin = 1;
bool cone_button_val = LOW;

const int limit_switch_pin = 3;
bool limit_switch_val = LOW;

void setup() {
  // put your setup code here, to run once:

  pinMode(encoder_pin, INPUT);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  attachInterrupt(digitalPinToInterrupt(encoder_pin), step_counter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_effect_pin), hall_effect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(cone_button_pin), cone_button, CHANGE);
  attachInterrupt(digitalPinToInterrupt(limit_switch_pin), limit_switch, CHANGE);

}

void hall_effect() {
//  hall_effect_val = !hall_effect_val;
  hall_effect_val = digitalRead(hall_effect_pin);
}

void cone_button() {
//  cone_button_val = !cone_button_val;
  cone_button_val = digitalRead(cone_button_pin);
}

void limit_switch() {
//  limit_switch_val = !limit_switch_val;
  limit_switch_val = digitalRead(limit_switch_pin);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(.001);
  Serial.print("cone button value"); Serial.print(cone_button_val); Serial.print("   hall effect value:"); Serial.print(hall_effect_val);
  Serial.print("   encoder value"); Serial.print(steps); Serial.print("   limit switch value:"); Serial.println(limit_switch_val);
//  if (counting == true)
//  {
//   Serial.print("number of steps"); Serial.println(steps); 
//  }

}

ISR (SPI_STC_vect)
{
  received = SPDR;

  if (received == 1)
  {
    counting = true;
  }
  else if (received == 2)
  {
    current_step = steps;
//    current_step_part1 = highByte(current_step);
    SPDR = highByte(current_step);
  }
  else if (received == 3)
  {
//    current_step_part2 = lowByte(current_step);
    SPDR = lowByte(current_step);
  }
  else if (received == 4)
  {
    counting = false;
    steps = 0;
    current_step = 0;
  }
  else if (received == 5) // This is the hall_effect_reading
  {
//    hall_effect = digitalRead(hall_effect_pin);
    if (hall_effect_val == HIGH)
    {
     SPDR = 1;
    }
    else
    {
      SPDR = 0;
    }
  }
  else if (received == 6)
  {
//    button_val = digitalRead(cone_button_pin);
    if (cone_button_val == HIGH)
    {
      SPDR = 1;
    }
    else
    {
      SPDR = 0;
    }
  }

  else if (received == 7)
  {
    if (limit_switch_val == HIGH)
    {
      SPDR = 1;
    }
    else 
    {
      SPDR = 0;
    }
  }
  
}

void step_counter()
{
  if (counting == true)
  {
    steps++;
  }
}
