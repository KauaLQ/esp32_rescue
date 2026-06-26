#ifndef MOTOR_CMD_H
#define MOTOR_CMD_H

#include <Arduino.h>

// Ponte H DRV8833
const int IN1 = 10;
const int IN2 = 7;
const int IN3 = 20;
const int IN4 = 21;

// Canais PWM
const int CH_IN1 = 2;
const int CH_IN2 = 3;
const int CH_IN3 = 4;
const int CH_IN4 = 5;

// Configuração PWM
const int PWM_FREQ = 1000;      // 1 kHz
const int PWM_RESOLUTION = 8;   // 0-255

const int MOTOR_SPEED = 120;    // PWM base dos motores

void configMotores();
void pararMotores();
void frente(int pwm = MOTOR_SPEED);
void re(int pwm = MOTOR_SPEED);
void giraDireita(int pwm = MOTOR_SPEED);
void giraEsquerda(int pwm = MOTOR_SPEED);
void setMotorPWM(int motor, int pwmA, int pwmB);

#endif