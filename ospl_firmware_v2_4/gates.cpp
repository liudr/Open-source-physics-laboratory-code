//112
#include "gates.h"
#include <Arduino.h>
/*
extern int storage_pointer;
extern entry storage[];
extern LiquidCrystal lcd;
*/
extern byte in_polarity;
extern byte out_polarity;

gates::gates(byte gp)
{
  t_triggered=t_released=trigger_count=0;
  gate_pin=gp;
  pinMode(gp, INPUT);
  digitalWrite(gp, HIGH);
}

void gates::reset()
{
  pinMode(gate_pin, INPUT);
  digitalWrite(gate_pin, HIGH);
  clear_counts();
  stat=not_ready;
}

byte gates::run()
{
    switch(stat)
    {
      case not_ready:
      do_not_ready();
      break;
      
      case ready:
      do_ready();
      break;
      
      case ready_to_triggered:
      do_ready_to_triggered();
      break;
      
      case held:
      do_held();
      break;
      
      case held_to_released:
      do_held_to_released();
      break;
      
      default:
      break;
    }
  return stat;
}  

void gates::do_not_ready()
{
  if (digitalRead(gate_pin)==(!in_polarity)) stat=ready;
}

void gates::do_ready()
{
  if (digitalRead(gate_pin)==in_polarity)
  {
    t_triggered=micros();
    trigger_count++;
    stat=ready_to_triggered;
  }
}

void gates::do_ready_to_triggered()
{
  if (digitalRead(gate_pin)==in_polarity) stat=held;
  else stat=ready;
}

void gates::do_held()
{
  if (digitalRead(gate_pin)==(!in_polarity))
  {
    t_released=micros();
    stat=held_to_released;
  }
//  else if (micros()-t_triggered>timeout_us) stat=time_out;
}

void gates::do_held_to_released()
{
  if (digitalRead(gate_pin)==(!in_polarity))
  {
    stat=ready;
  }
  else stat=held;
}

unsigned long gates::get_counts()
{
  return trigger_count;
}

void gates::set_counts(unsigned long ct)
{
  trigger_count=ct;
}

void gates::clear_counts()
{
  trigger_count=0;
}

