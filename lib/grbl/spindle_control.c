#include "grbl.h"

// TOGGLE this when installed solenoid-controlled spray gun
// void spindle_init()
// {    
//   SPINDLE_ENABLE_DDR |= (1 << SPINDLE_ENABLE_BIT);
//   spindle_stop();
// }

// void spindle_stop()
// {
//   SPINDLE_ENABLE_PORT &= ~(1 << SPINDLE_ENABLE_BIT);
// }

// void spindle_run(uint8_t direction, float rpm) 
// {
//   if (sys.state == STATE_CHECK_MODE) { return; }
  
//   protocol_auto_cycle_start(); 
//   protocol_buffer_synchronize(); 
  
//   if (direction == SPINDLE_DISABLE) {
//     spindle_stop();
//   } else {
//     SPINDLE_ENABLE_PORT |= (1 << SPINDLE_ENABLE_BIT);
//   }
// }

// // Hàm này cần thiết cho GRBL phiên bản mới, ngay cả khi để trống
// void spindle_set_state(uint8_t state, float rpm) {
//    if (state == SPINDLE_DISABLE) {
//       spindle_stop();
//    } else {
//       spindle_run(state, rpm);
//    }
// }


// TOGGLE this when installed a servo-controlled spray gun

#define SERVO_PERIOD_TICKS 39999 // Prescaler = 8 (0.5us/tick) => 20ms (50Hz) = 40000 ticks

#define SERVO_MIN_PULSE 1000 // 0 deg
#define SERVO_MAX_PULSE 5000 // 180 deg

uint16_t servo_min_pos = 0;

void spindle_init()
{    
  // config targeted pin as output
  SPINDLE_ENABLE_DDR |= (1 << SPINDLE_ENABLE_BIT);

  // Reset timer 4
  TCCRA_REGISTER = 0;
  TCCRB_REGISTER = 0;

  // Initialize Mode 14 (Fast PWM, TOP = ICR4)
  // WGM43=1, WGM42=1, WGM41=1, WGM40=0
  TCCRA_REGISTER |= (1 << WGM41);
  TCCRB_REGISTER |= (1 << WGM43) | (1 << WGM42);

  // Initialize Output Compare Mode for channel A (OC4A)
  // Clear OC4A on Compare Match, Set OC4A at Bottom (Non-inverting)
  TCCRA_REGISTER |= (1 << COM4A1);

  // Set operating freq at 50Hz
  ICR_REGISTER = SERVO_PERIOD_TICKS;

  // Set Prescaler to 8
  TCCRB_REGISTER |= (1 << CS41);

  spindle_stop();
}

void spindle_stop()
{
  uint16_t pwm_value = SERVO_MIN_PULSE + (uint16_t)( ( SPINDLE_MAX_RPM * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180.0 );
  OCR_REGISTER = pwm_value; // setting servo back to 0 (change this to 90)
}

void spindle_run(uint8_t direction, float rpm) 
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  
  protocol_auto_cycle_start(); 
  protocol_buffer_synchronize(); 
  
  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {
    float current_rpm = rpm;

    // limit servo range
    if (current_rpm > SPINDLE_MAX_RPM) current_rpm = SPINDLE_MAX_RPM;
    if (current_rpm < SPINDLE_MIN_RPM) current_rpm = SPINDLE_MIN_RPM;

    // mapping servo angle to raw value
    uint16_t pwm_value = SERVO_MIN_PULSE + (uint16_t)( (current_rpm * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180.0 );

    // write out to OCR register
    OCR_REGISTER = pwm_value;
  }
}

// Hàm này cần thiết cho GRBL phiên bản mới, ngay cả khi để trống
void spindle_set_state(uint8_t state, float rpm) {
   if (state == SPINDLE_DISABLE) {
      spindle_stop();
   } else {
      spindle_run(state, rpm);
   }
}