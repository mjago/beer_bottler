#include <MsTimer2.h>

/* Defines */

#define CAN_PUSH_TIME        2000 /* 2 seconds */
#define SETTLING_TIME        2000 /* 2 seconds */
#define MAX_FILL_TIME        6000 /* 6 seconds */
#define PURGE_TIME           4000 /* 4 seconds */

#if PURGE_TIME > MAX_FILL_TIME
#error PURGE_TIME must be less than MAX_FILL_TIME
#endif

#define ONE_SECOND           1000 /* 1 second */
#define FILL_RAIL_DELAY_UP   ONE_SECOND
#define FILL_RAIL_DELAY_DOWN ONE_SECOND

#define CAN_PUSH_SLD      2    /* Can Push    Solenoid */
#define FILL_RAIL_SLD     3    /* Fill Rail   Solenoid */
#define PURGE_VALVE_SLD   4    /* Purge Valve Solenoid */
#define BEER_VALVE_SLD_0  5    /* Beer Valve  Solenoid */
#define BEER_VALVE_SLD_1  6    /* Beer Valve  Solenoid */
#define BEER_VALVE_SLD_2  7    /* Beer Valve  Solenoid */

#define FILL_SENSE_0      A0   /* input to detect when can is full. */
#define FILL_SENSE_1      A1   /* input to detect when can is full. */
#define FILL_SENSE_2      A2   /* input to detect when can is full. */
#define LED_PIN           13

#define RAISE             1
#define LOWER             0
#define OPEN              1
#define CLOSE             0
#define START             1
#define STOP              0
#define ON                1
#define OFF               0
#define FULL              0
#define CANS_THIS_RUN    30   /* Number of Cans to Can */
#define CAN_COUNT         3   /* Number of Cans filled simultaneously */

/* Process States */

typedef enum
{
  IDLE,
  CAN_PUSH,
  LOWER_FILL_RAIL,
  SETTLING_DELAY,
  OPEN_VALVES,
  MONITOR_FILLING,
  RAISE_FILL_RAIL,
  CHECK_RUN
} process_t;

typedef enum
{
  FILLING,
  ALL_FULL,
  FILL_TIMER_EXPIRED
} fill_state_t;

/* Function Prototypes */

void elapse_timer_interrupt(void);
void set_timer(int millisec);
void stop_timer(void);
bool timer_elapsed(void);
void process_to(process_t proc);
bool idle(void);
void can_push(bool start_stop);
void fill_rail(bool up_down);
fill_state_t fill_progress(void);
bool check_all_full(void);
bool can_full(int can_number);
void operate_all_beer_valves(int action);
void abort_process(void);
void operate_valve(int type, bool action);
void check_beer_sensors(void);
void flash_led(void);
const char * process_string(int proc);

/* Variables */

process_t     process;
int           cans_processed;
volatile bool elapsed = false;
int fill_sensor[CAN_COUNT] = {
  FILL_SENSE_0,
  FILL_SENSE_1,
  FILL_SENSE_2
};

int beer_valve[CAN_COUNT] = {
  BEER_VALVE_SLD_0,
  BEER_VALVE_SLD_1,
  BEER_VALVE_SLD_2
};

/* Setup */

void setup(void)
{
  int count;

  Serial.begin(9600);
  Serial.println("Starting Up.");
  pinMode(CAN_PUSH_SLD, OUTPUT);
  pinMode(FILL_RAIL_SLD, OUTPUT);
  pinMode(PURGE_VALVE_SLD, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  for(count = 0; count < CAN_COUNT; count++)
  {
    pinMode(beer_valve[count], OUTPUT);
    pinMode(fill_sensor[count], INPUT_PULLUP);
  }

  flash_led();
//  check_beer_sensors();
//  cans_processed = 0;
//  operate_valve(PURGE_VALVE_SLD, CLOSE);
//  operate_all_beer_valves(CLOSE);
//  fill_rail(RAISE);
//  process_to(IDLE);
}

/* Main Loop */

void loop(void)
{
  for(;;)
  {
    int level[CAN_COUNT];
    int calibration = 400;

    for(int count = 0; count < CAN_COUNT; count++)
    {
      level[count] = analogRead(fill_sensor[count]);
      Serial.print("Level Is ");
      Serial.println(level[count]);

      if(level[count] < calibration)
        Serial.println("Triggered!");

      delay(100);
    }
    Serial.println("\n");
      delay(100);
  }

  fill_state_t fill_state;

  switch(process)
  {

  case IDLE:
    if(idle() == false)
    {
      set_timer(CAN_PUSH_TIME);
      can_push(START);
      process_to(CAN_PUSH);
    }
    break;

  case CAN_PUSH:
    if(timer_elapsed())
    {
      can_push(STOP);
      process_to(LOWER_FILL_RAIL);
    }
    break;

  case LOWER_FILL_RAIL:
    fill_rail(LOWER);
    set_timer(SETTLING_TIME);
    process_to(SETTLING_DELAY);
    break;

  case SETTLING_DELAY:
    if(timer_elapsed())
    {
      process_to(OPEN_VALVES);
    }
    break;

  case OPEN_VALVES:
    operate_valve(PURGE_VALVE_SLD, OPEN);
    operate_all_beer_valves(OPEN);
    set_timer(PURGE_TIME);
    process_to(MONITOR_FILLING);
    break;

  case MONITOR_FILLING:
    fill_state = fill_progress();
    switch(fill_state)
    {
    case FILLING:
      break;

    case ALL_FULL:
      stop_timer();
      process_to(RAISE_FILL_RAIL);
      break;

    case FILL_TIMER_EXPIRED:
      abort_process();
      break;

    default:
      abort_process();
      break;
    }
    break;

  case RAISE_FILL_RAIL:
    Serial.println("  Raising Fill Rail");
    fill_rail(RAISE);
    process_to(CHECK_RUN);
    break;

  case CHECK_RUN:
    Serial.println("  Completed Cycle");
    cans_processed += CAN_COUNT;
    if(cans_processed >= CANS_THIS_RUN)
    {
      Serial.print("  Run Complete\n  ");
      Serial.print(cans_processed);
      Serial.println(" Processed");
      /* Infinite loop */
      for(;;);
    }
    else
    {
      Serial.print("  ");
      Serial.print(cans_processed);
      Serial.println("  Cans Filled.\n");
    }
    process_to(IDLE);
    break;

  default:
    abort_process();
    break;
  }
}

/* Functions */

void elapse_timer_interrupt(void)
{
  elapsed = true;
}

void set_timer(int millisec)
{
  MsTimer2::set(millisec, elapse_timer_interrupt);
  MsTimer2::start();
}

void stop_timer(void)
{
  MsTimer2::stop();
}

/* Return true if timer has elapsed */
bool timer_elapsed(void)
{
  bool ret = false;

  if(elapsed)
  {
    stop_timer();
    elapsed = false;
    ret = true;
  }

  return ret;
}

/* Move to different process */
void process_to(process_t proc)
{
  Serial.print("Process -> ");
  Serial.println(process_string(proc));

  process = proc;
}

bool idle(void)
{
  bool ret = true;

  if(cans_processed < CANS_THIS_RUN)
  {
    ret = false;
  }
  else
  {
    /* Start Switch? */
  }
  return ret;
}

void can_push(bool start_stop)
{
  if(start_stop == START)
  {
    Serial.println("  Pushing Cans...");
  }
  else
  {
    Serial.println("  Stop Pushing Cans");
  }

  digitalWrite(CAN_PUSH_SLD, start_stop);
}

void fill_rail(bool up_down)
{
  digitalWrite(FILL_RAIL_SLD, up_down);

  if(up_down)
  {
    Serial.println("  Moving fill rail Up");
    delay(FILL_RAIL_DELAY_UP);
  }
  else
  {
    Serial.println("  Moving fill rail Down");
    delay(FILL_RAIL_DELAY_DOWN);
  }
}

fill_state_t fill_progress(void)
{
  static bool purge_complete = false;
  fill_state_t fill_state;
  int can_number;
  int full_count;

  if(check_all_full() == true)
  {
    Serial.println("  All Cans Full!");
    fill_state = ALL_FULL;
    purge_complete = false;
  }

  else if(timer_elapsed())
  {
    if(purge_complete == false)
    {
      Serial.println("  CO2 Purge Complete");
      purge_complete = true;
      set_timer(MAX_FILL_TIME - PURGE_TIME);
    }
    else
    {
      Serial.println("  Fill Guard Time Exceeded!");
      purge_complete = false;
      operate_all_beer_valves(CLOSE);
      fill_state = FILL_TIMER_EXPIRED;
    }
  }

  else
  {
    fill_state = FILLING;
  }

  return fill_state;
}

bool check_all_full(void)
{
  int can_number = 0;
  int full_count = 0;
  bool all_full = false;

  for(can_number = 0; can_number < CAN_COUNT; can_number++)
  {
    if(can_full(can_number))
    {
      full_count += 1;
    }
  }

  if (full_count >= CAN_COUNT)
  {
    all_full = true;
  }

  return all_full;
}

bool can_full(int can_number)
{
  bool ret = false;
  int  level;

  /* read the Beer Sensor */

  level = analogRead(fill_sensor[can_number]);
  Serial.print("Can number ");
  Serial.print(can_number);
  Serial.print(", Level Is ");
  Serial.println(level);

  if(0)// != FULL)
  {
    operate_valve(beer_valve[can_number], CLOSE);
    Serial.print("  Can number ");
    Serial.print(can_number);
    Serial.println(" Is Full!");
    ret = true;
  }

  return ret;
}

void operate_all_beer_valves(int action)
{
  int can_number;

  if(action == OPEN)
  {
    Serial.println("  Opening All Beer Valves");
  }
  else
  {
    Serial.println("  Closing All Beer Valves");
  }

  for(can_number = 0; can_number < CAN_COUNT; can_number++)
  {
    operate_valve(beer_valve[can_number], action);
  }
}

void abort_process(void)
{
  Serial.print("\nProcess Aborted (");
  Serial.print(process_string(process));
  Serial.println(")");

  operate_all_beer_valves(CLOSE);
  operate_valve(PURGE_VALVE_SLD, CLOSE);
  fill_rail(RAISE);
  Serial.println("  Stopped!");

  for(;;)
  {
    flash_led();
    delay(500);
  }
}

void operate_valve(int type, bool action)
{
  digitalWrite(type, action);
}

void check_beer_sensors(void)
{
  uint8_t count;
  bool    ready;

  /* Check No beer sensors are Submerged! */
  for(;;)
  {
    ready = true;
    for(count = 0; count < CAN_COUNT; count++)
    {
      if(digitalRead(fill_sensor[count]) == FULL)
      {
        ready = false;
      }
    }
    if(ready)
    {
      break;
    }

    Serial.println("A Beer Sensor is Submerged!");
    delay(ONE_SECOND);
  }
}

void flash_led(void)
{
  int count;
  int pin;

  for(count = 0; count < 4; count++)
  {
    pin = (pin == ON) ? OFF : ON;
    digitalWrite(LED_PIN, pin);
    delay(50);
  }
  digitalWrite(LED_PIN, ON);
}

const char * process_string(int proc)
{
  const char * process_str[] =
    {
      "IDLE",
      "CAN_PUSH",
      "LOWER_FILL_RAIL",
      "SETTLING_DELAY",
      "OPEN_VALVES",
      "MONITOR_FILLING",
      "RAISE_FILL_RAIL",
      "CHECK_RUN"
    };
  return process_str[proc];
}
