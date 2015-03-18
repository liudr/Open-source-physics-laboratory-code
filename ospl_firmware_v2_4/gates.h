//34
#ifndef gates_h
#define gates_h
#include <Arduino.h>
#define not_ready 0
#define ready 1
#define ready_to_triggered 2
#define held 3
#define held_to_released 4
#define time_out 5
#define disabled 99
//#define timeout_us 500000
class gates
{
  public:
  unsigned long t_triggered;
  unsigned long t_released;
  unsigned long trigger_count;
  byte stat;
  byte gate_pin;
  gates(byte gp);
  void reset();
  byte run();
  void do_not_ready();
  void do_ready();
  void do_ready_to_triggered();
  void do_held();
  void do_held_to_released();
  void clear_counts();
  unsigned long get_counts();
  void set_counts(unsigned long ct);
};

#endif
