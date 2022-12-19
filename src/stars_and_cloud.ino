const uint16_t pwmtable_8D[32] PROGMEM = {
  0, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 8, 10, 11, 13, 16, 19, 23,
  27, 32, 38, 45, 54, 64, 76, 91, 108, 128, 152, 181, 215, 255
};

typedef struct led {
  int pin;
  uint16_t pwm_level;         // 0 .. 31
  uint16_t pwm_level_before;  // level before flickering started
  int step;                   // -1  getting darker, or 1 getting brighter
  bool in_flicker_mode;       // is currently flickering
};


const int FULL_ON = 0;
const int FLICKER_TO_DIMMED = 1;
const int FLICKER_TO_FULL = 2;
const int FLICKER_TO_RECENT = 3;
const int DIMMED_ON = 4;
const int OFF = 5;  // only for cloud


const int PWM_MAX = 31;
const int PWM_DIMMED = 15;



typedef struct timer {
  bool running;
  unsigned long start_time;
};

typedef struct finiteStateMachine {
  int state;
  int nextstep;
  int wait_time_in_step;
  int flicker_time;
  int full_level;
  int dimmed_level;
  struct led *led;       // Star LED or Blue LED in cloud
  struct led *ledRed;    // only for cloud
  struct led *ledGreen;  // only for cloud
  struct timer *timer;
};

struct led ledStarWhite1 = { 6, 0, 0, -1, false };
struct led ledStarWhite2 = { 5, 0, 0, -1, false };
struct led ledStarYellow = { 4, 0, 0, -1, false };
struct led ledBlue = { 16, 0, 0, -1, false };
struct led ledRed = { 17, 0, 0, -1, false };
struct led ledGreen = { 18, 0, 0, -1, false };

struct timer timer_white1 = { false, 0 };
struct timer timer_white2 = { false, 0 };
struct timer timer_yellow = { false, 0 };
struct timer timer_cloud = { false, 0 };



struct finiteStateMachine fsm_white1 = { FULL_ON, 0, 2000, 25, PWM_MAX, PWM_DIMMED, &ledStarWhite1, 0, 0, &timer_white1 };
struct finiteStateMachine fsm_white2 = { FULL_ON, 0, 2000, 25, PWM_MAX, PWM_DIMMED, &ledStarWhite2, 0, 0, &timer_white2 };
struct finiteStateMachine fsm_yellow = { FULL_ON, 0, 2000, 25, PWM_MAX, PWM_DIMMED, &ledStarYellow, 0, 0, &timer_yellow };

struct finiteStateMachine fsmCloud = { OFF, 0, 2000, 5, PWM_MAX, PWM_DIMMED, &ledBlue, &ledRed, &ledGreen, &timer_cloud };



void setup() {
  Serial.begin(9600);
  Serial.println("hello from pico...");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledBlue.pin, OUTPUT);
  pinMode(ledRed.pin, OUTPUT);
  pinMode(ledGreen.pin, OUTPUT);
  pinMode(ledStarYellow.pin, OUTPUT);
  pinMode(ledStarWhite1.pin, OUTPUT);
  pinMode(ledStarWhite2.pin, OUTPUT);

  Serial.println(analogRead(0));
  randomSeed(analogRead(0));
}

void loop() {

  fsm_star(&fsm_white1);
  fsm_star(&fsm_white2);
  fsm_star(&fsm_yellow);

  fsm_cloud(&fsmCloud);

  // write all LED outputs
  writeLed(&ledBlue);
  writeLed(&ledRed);
  writeLed(&ledGreen);
  writeLed(&ledStarYellow);
  writeLed(&ledStarWhite1);
  writeLed(&ledStarWhite2);
}

// flicker ramps up and/or down the led brightness to the given limit
// if flicker_to_recent_value is set, it will ramp up and down
// will return true when flickering is done
bool flicker(led *led, timer *t, int flicker_speed, int limit, bool flicker_to_recent_value) {
  if (led->in_flicker_mode) {
    // flickering started already
    if (ton_timer(t, flicker_speed)) {
      led->pwm_level += led->step;
      if (flicker_to_recent_value) {
        if (led->pwm_level == limit) {
          led->step *= -1;
        }
        if (led->pwm_level == led->pwm_level_before) {
          led->in_flicker_mode = false;
          return true;
        }
      } else {
        if (led->pwm_level == limit) {
          // finished flickering up or down
          led->in_flicker_mode = false;
          return true;
        }
      }
      // Serial.println("led->step");
      // Serial.println(led->step);
      // Serial.println("led->pwm_level");
      // Serial.println(led->pwm_level);
      // Serial.println("led->pwm_level_before");
      // Serial.println(led->pwm_level_before);

      return false;
    }
  } else {
    // not yet flickering, start it..
    led->in_flicker_mode = true;
    led->pwm_level_before = led->pwm_level;
    if (limit < led->pwm_level) {
      led->step = -1;
    } else {
      led->step = 1;
    }
    return false;
  }
}

// flash will flash the led (for the cloud)
bool flash(led *led, timer *t, int flash_time) {
  if (led->in_flicker_mode) {
    // flickering started already
    if (ton_timer(t, flash_time)) {
      led->pwm_level = 0;
      led->in_flicker_mode = false;

      Serial.println(analogRead(0));
      randomSeed(analogRead(0));
      
      return true;
    }
    return false;
  } else {
    // not yet flashing, start it..
    led->in_flicker_mode = true;
    led->pwm_level = PWM_MAX;
    return false;
  }
}


void writeLed(led *led) {
  analogWrite(led->pin, pwmtable_8D[led->pwm_level]);
}

// returns false for half the duration_ms time and true for the other half
// but the start is random
bool ticker(int duration_ms) {
  return (millis() % duration_ms > (duration_ms / 2));
}

bool ton_timer(timer *t, int wait_time) {
  if (t->running) {
    if (millis() - t->start_time >= wait_time) {
      // time is up
      t->running = false;
      return true;
    }
    return false;
  }
  // start timer
  t->running = true;
  t->start_time = millis();
}

void fsm_cloud(finiteStateMachine *fsm) {

  switch (fsm->state) {
    case OFF:

      fsm->led->pwm_level = 0;
      fsm->ledRed->pwm_level = 0;
      fsm->ledGreen->pwm_level = 0;



      if (ton_timer(fsm->timer, fsm->wait_time_in_step)) {
        fsm->nextstep = random(1, 101);  // 1 .. 100
        if (fsm->nextstep <= 33) {       // 33%
          fsm->nextstep = FLICKER_TO_RECENT;
        } else {
          fsm->nextstep = OFF;          
        }

        fsm->flicker_time = random(100, 200);  //random(18, 32);

        fsm->wait_time_in_step = random(15000, 30000);


        fsm->state = fsm->nextstep;
        Serial.print("cloud OFF-> ");
        Serial.println(fsm->nextstep);
        Serial.println(fsm->wait_time_in_step);
      }
      break;

    case FLICKER_TO_RECENT:
      if (flash(fsm->led, fsm->timer, fsm->flicker_time)) {
        fsm->state = OFF;
        Serial.print("FLICKER_TO_RECENT -> ");
        Serial.println(fsm->state);
      }

      fsm->ledRed->pwm_level = fsm->led->pwm_level;
      if (fsm->led->pwm_level > 3) {
        fsm->ledGreen->pwm_level = fsm->led->pwm_level - 3;
      } else {
        fsm->ledGreen->pwm_level = fsm->led->pwm_level;
      }
      break;

    default:
      fsm->state = OFF;
      break;
  }
}

void fsm_star(finiteStateMachine *fsm) {

  switch (fsm->state) {
    case FULL_ON:

      fsm->led->pwm_level = fsm->full_level;

      if (ton_timer(fsm->timer, fsm->wait_time_in_step)) {
        fsm->nextstep = random(1, 101);  // 1 .. 100
        if (fsm->nextstep <= 10) {       // 10%
          fsm->nextstep = FLICKER_TO_DIMMED;
        } else if (fsm->nextstep <= 66) {
          fsm->nextstep = FULL_ON;
        } else {  // 33 %
          fsm->nextstep = FLICKER_TO_RECENT;
        }
        fsm->flicker_time = random(18, 32);
        fsm->dimmed_level = random(13, 19);
        fsm->wait_time_in_step = random(1500, 2500);

        fsm->state = fsm->nextstep;
        // Serial.print("FULL_ON -> ");
        // Serial.println(fsm->nextstep);
      }
      break;

    case FLICKER_TO_RECENT:
      if (flicker(fsm->led, fsm->timer, fsm->flicker_time, fsm->dimmed_level, true)) {
        fsm->state = FULL_ON;
        // Serial.print("FLICKER_TO_RECENT -> ");
        // Serial.println(fsm->state);
      }


      break;

    case FLICKER_TO_DIMMED:
      if (flicker(fsm->led, fsm->timer, fsm->flicker_time, fsm->dimmed_level, false)) {
        fsm->state = DIMMED_ON;
        // Serial.print("FLICKER_TO_DIMMED -> ");
        // Serial.println(fsm->state);
      }
      break;

    case DIMMED_ON:
      fsm->led->pwm_level = fsm->dimmed_level;

      if (ton_timer(fsm->timer, fsm->wait_time_in_step)) {
        fsm->nextstep = random(1, 101);  // 1 .. 100
        if (fsm->nextstep <= 33) {       // 33%
          fsm->nextstep = DIMMED_ON;
        } else {  // 33 %
          fsm->nextstep = FLICKER_TO_FULL;
        }
        fsm->flicker_time = random(18, 32);

        fsm->state = fsm->nextstep;
        // Serial.print("DIMMED_ON -> ");
        // Serial.println(fsm->nextstep);
      }
      break;

    case FLICKER_TO_FULL:
      if (flicker(fsm->led, fsm->timer, fsm->flicker_time, fsm->full_level, false)) {
        fsm->state = FULL_ON;
        // Serial.print("FLICKER_TO_FULL -> ");
        // Serial.println(fsm->state);
      }
      break;

    default:
      fsm->state = FULL_ON;
  }
}
