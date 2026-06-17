#include "motor_cmd.h"

void configMotores() {
    // Configura os canais PWM
    ledcSetup(CH_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_IN3, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_IN4, PWM_FREQ, PWM_RESOLUTION);

    // Associa os canais aos pinos
    ledcAttachPin(IN1, CH_IN1);
    ledcAttachPin(IN2, CH_IN2);
    ledcAttachPin(IN3, CH_IN3);
    ledcAttachPin(IN4, CH_IN4);

    // Garante tudo desligado
    ledcWrite(CH_IN1, 0);
    ledcWrite(CH_IN2, 0);
    ledcWrite(CH_IN3, 0);
    ledcWrite(CH_IN4, 0);
}

void pararMotores() {
    ledcWrite(CH_IN1, 0);
    ledcWrite(CH_IN2, 0);
    ledcWrite(CH_IN3, 0);
    ledcWrite(CH_IN4, 0);
}

void frente(int pwm) {
    ledcWrite(CH_IN1, pwm);
    ledcWrite(CH_IN2, 0);
    ledcWrite(CH_IN3, pwm);
    ledcWrite(CH_IN4, 0);
}

void re(int pwm) {
    ledcWrite(CH_IN1, 0);
    ledcWrite(CH_IN2, pwm);
    ledcWrite(CH_IN3, 0);
    ledcWrite(CH_IN4, pwm);
}

void giraDireita(int pwm) {
    ledcWrite(CH_IN1, pwm);
    ledcWrite(CH_IN2, 0);
    ledcWrite(CH_IN3, 0);
    ledcWrite(CH_IN4, pwm);
}

void giraEsquerda(int pwm) {
    ledcWrite(CH_IN1, 0);
    ledcWrite(CH_IN2, pwm);
    ledcWrite(CH_IN3, pwm);
    ledcWrite(CH_IN4, 0);
}