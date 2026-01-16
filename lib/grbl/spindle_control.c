#include "grbl.h"

/* * Cấu hình Servo cho Arduino Mega 2560 - Timer 4 - Chân D6 (PH3/OC4A)
 * Tần số: 50Hz (Chu kỳ 20ms)
 * Prescaler: 8 (0.5us per tick)
 */

// Định nghĩa giới hạn xung Servo (tính theo số tick timer 0.5us)
// 1ms = 2000 ticks, 2ms = 4000 ticks
#define SERVO_MIN_PULSE     1000  
#define SERVO_MAX_PULSE     2700 
#define SERVO_PERIOD_TICKS  39999 // (20ms / 0.5us) - 1

void spindle_init()
{    
  // 1. Cấu hình chân D6 (PH3) là Output
  // Lưu ý: Đảm bảo macro SPINDLE_PWM_DDR/BIT trỏ đúng về PH3 trong file cpu_map.h
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); 

  // Các chân Enable/Direction khác (giữ nguyên logic cũ)
  #if defined(CPU_MAP_ATMEGA2560_RAMPS_1_4) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  #else  
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); 
  #endif
  
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  #endif

  // 2. Khởi tạo Timer 4 cho Servo
  // Xóa các thanh ghi điều khiển
  TCCRA_REGISTER = 0;
  TCCRB_REGISTER = 0;

  // Cấu hình Mode 14 (Fast PWM, TOP = ICR4)
  // WGM43=1, WGM42=1, WGM41=1, WGM40=0
  TCCRA_REGISTER |= (1<<WAVE1_REGISTER);
  TCCRB_REGISTER |= (1<<WAVE3_REGISTER) | (1<<WAVE2_REGISTER);

  // Cấu hình Output Compare Mode (Non-inverting trên kênh A)
  // Xóa bit khi khớp, Set lại khi về đáy
  TCCRA_REGISTER |= (1<<COMB_BIT);

  // Đặt chu kỳ xung 50Hz (20ms) vào ICR4
  ICR_REGISTER = SERVO_PERIOD_TICKS;

  // Cấu hình Prescaler = 8
  // CS41 = 1
  TCCRB_REGISTER |= (1<<CS41);

  spindle_stop();
}

void spindle_stop()
{
    // Đưa về vị trí thấp nhất (hoặc tắt hẳn tùy servo)
    OCR_REGISTER = SERVO_MIN_PULSE; 
}

void spindle_run(uint8_t direction, float rpm) 
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  
  protocol_auto_cycle_start(); 
  protocol_buffer_synchronize(); 
  
  if (direction == SPINDLE_DISABLE) {
    spindle_stop();
  } else {
    // Xử lý chân hướng (Direction Pin)
    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (direction == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif
  
    // Tính toán giá trị PWM cho Servo
    // RPM Range: 0 -> SPINDLE_MAX_RPM
    // PWM Range: 2000 -> 4000
    
    float current_rpm = rpm;
    if (current_rpm > SPINDLE_MAX_RPM) current_rpm = SPINDLE_MAX_RPM;
    if (current_rpm < SPINDLE_MIN_RPM) current_rpm = SPINDLE_MIN_RPM;

    // Map giá trị RPM sang độ rộng xung
    // Công thức: PWM = MIN + (RPM * (MAX - MIN) / MAX_RPM)
    uint16_t pwm_value = SERVO_MIN_PULSE + (uint16_t)( (current_rpm * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / SPINDLE_MAX_RPM );

    // Ghi vào thanh ghi so sánh kênh A (D6)
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