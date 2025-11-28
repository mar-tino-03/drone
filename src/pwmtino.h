#ifndef _PWMTINO_H_
#define _PWMTINO_H_

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_timer.h"

//hw_timer_t * timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

esp_timer_handle_t timer_pwm;
volatile uint32_t tick = 0;

volatile int mot_1, mot_2, mot_3, mot_4;

// Struttura per passare i parametri alla ISR
typedef struct {
  int pin[4];
  int freq;
} pwm_args_t;
static pwm_args_t pwmArgs;

// -------------------------------------------------------------

void IRAM_ATTR onTimer_pwm(void* arg) 
{
  pwm_args_t* p = (pwm_args_t*)arg;

  tick = micros();

  const uint32_t period_us = 1000000UL / p->freq;
  const uint32_t phaseShift_us = period_us / 4;

  uint32_t t = tick % period_us;

  // Sezione critica
  uint16_t m1, m2, m3, m4;
  portEXIT_CRITICAL_ISR(&timerMux);
    m1 = mot_1;
    m2 = mot_2;
    m3 = mot_3;
    m4 = mot_4;
  portEXIT_CRITICAL_ISR(&timerMux);

  uint32_t duty1 = (uint32_t)m1 * period_us / 1023;
  uint32_t duty2 = (uint32_t)m2 * period_us / 1023;
  uint32_t duty3 = (uint32_t)m3 * period_us / 1023;
  uint32_t duty4 = (uint32_t)m4 * period_us / 1023;

  uint32_t t1 = t;
  uint32_t t2 = (t + period_us - phaseShift_us) % period_us;
  uint32_t t3 = (t + period_us - 2 * phaseShift_us) % period_us;
  uint32_t t4 = (t + period_us - 3 * phaseShift_us) % period_us;

  gpio_set_level((gpio_num_t)p->pin[0], (t1 >= duty1) ? 1 : 0);
  gpio_set_level((gpio_num_t)p->pin[1], (t2 >= duty2) ? 1 : 0);
  gpio_set_level((gpio_num_t)p->pin[2], (t3 >= duty3) ? 1 : 0);
  gpio_set_level((gpio_num_t)p->pin[3], (t4 >= duty4) ? 1 : 0);
}

// -------------------------------------------------------------

void init_pwm(int pin1, int pin2, int pin3, int pin4, int freq, bool p){
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  mot_1 = 0;
  mot_2 = 0;
  mot_3 = 0;
  mot_4 = 0;

  
  if(p == true){

    gpio_set_level((gpio_num_t)pin1, 1);
    gpio_set_level((gpio_num_t)pin2, 1);
    gpio_set_level((gpio_num_t)pin3, 1);
    gpio_set_level((gpio_num_t)pin4, 1);

    // Popolo la struttura
    pwmArgs.pin[0] = pin1;
    pwmArgs.pin[1] = pin2;
    pwmArgs.pin[2] = pin3;
    pwmArgs.pin[3] = pin4;
    pwmArgs.freq   = freq;

    const esp_timer_create_args_t periodic_timer_pwm_args = {
      .callback = &onTimer_pwm,
      .arg = &pwmArgs,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "MyTimer_pwm"
    };

    esp_timer_create(&periodic_timer_pwm_args, &timer_pwm);

    uint32_t period_timer_us = 1000000UL / freq / 1000;

    esp_timer_start_periodic(timer_pwm, period_timer_us);
  }
  else{
    ledcSetup(0, freq, 10);
    ledcSetup(1, freq, 10);
    ledcSetup(2, freq, 10);
    ledcSetup(3, freq, 10);
    ledcAttachPin(pin1, 0);
    ledcAttachPin(pin2, 1);
    ledcAttachPin(pin3, 2);
    ledcAttachPin(pin4, 3);
  
    ledcWrite(0, 1023);
    ledcWrite(1, 1023);
    ledcWrite(2, 1023);
    ledcWrite(3, 1023);
  }
}



void set_power(int m1, int m2, int m3, int m4){
  mot_1 = m1;
  mot_2 = m2;
  mot_3 = m3;
  mot_4 = m4;

  if(phase==0){
    ledcWrite(0, 1023 - mot_1);
    ledcWrite(1, 1023 - mot_2);
    ledcWrite(2, 1023 - mot_3);
    ledcWrite(3, 1023 - mot_4);
  }
}

#endif
