#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "control_index.h"
#include "motor_cmd.h"
#include "env.h" // Lembre-se de criar seu próprio arquivo env.h se for usar o bot do Telegram, ou de substituir as variáveis botToken e chatId pelos seus valores reais.

const char* ssid = "CLEUDO";
const char* password = "91898487";

AsyncWebServer server(80);
WiFiUDP udp;
const int udpPort = 4210;
char incomingPacket[255];

#define BLINK 8

Servo servoX;
Servo servoY;
const int servoPinX = 2;
const int servoPinY = 3;
float posX = 60;
float posY = 60;
const int SERVO_RIGHT_LIMIT = 170;
const int SERVO_LEFT_LIMIT  = 10;
const float AREA_SETPOINT = 40000;
const float AREA_DEADBAND = 5000;

// --- Parâmetros PID ---
volatile float KpX = 0.01;
volatile float KiX = 0.000;
volatile float KdX = 0.00;
float errorX = 0;
float previousErrorX = 0;
float integralX = 0;

volatile float KpY = 0.02;
volatile float KiY = 0.000;
volatile float KdY = 0.00;
float errorY = 0;
float previousErrorY = 0;
float integralY = 0;

float targetArea = 0;
int targetId = -1;
unsigned long previousTime = 0;
unsigned long lastPacketTime = 0;

void enviarTelegram(const char *ip) {
    HTTPClient http;

    String url =
      "https://api.telegram.org/bot" +
      botToken +
      "/sendMessage?chat_id=" +
      chatId +
      "&text=" +
      ip;

    http.begin(url);
    int httpCode = http.GET();

    http.end();
}

void setup() {
    Serial.begin(115200);
    pinMode(BLINK, OUTPUT);
    bool ledState = false;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        ledState = !ledState;
        digitalWrite(BLINK, ledState);
    }

    IPAddress ip = WiFi.localIP();
    char ipStr[16];
    snprintf(ipStr, sizeof(ipStr), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    enviarTelegram(ipStr);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", htmlPage());
    });

    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        if(request->hasParam("kpx"))
            KpX = request->getParam("kpx")->value().toFloat();

        if(request->hasParam("kix"))
            KiX = request->getParam("kix")->value().toFloat();

        if(request->hasParam("kdx"))
            KdX = request->getParam("kdx")->value().toFloat();

        if(request->hasParam("kpy"))
            KpY = request->getParam("kpy")->value().toFloat();

        if(request->hasParam("kiy"))
            KiY = request->getParam("kiy")->value().toFloat();

        if(request->hasParam("kdy"))
            KdY = request->getParam("kdy")->value().toFloat();

        request->redirect("/");
    });

    digitalWrite(BLINK, LOW);

    server.begin();
    udp.begin(udpPort);

    configMotores();

    servoX.attach(servoPinX);
    servoY.attach(servoPinY);
    servoX.write(posX);
    servoY.write(posY);

    delay(1000);

    previousTime = millis();
}

void loop() {
    // --- Desliga motores se não receber dados por um tempo ---
    if (millis() - lastPacketTime > 1000) {
        pararMotores();
    }

    int packetSize = udp.parsePacket();

    if (packetSize) {
        lastPacketTime = millis();
        int len = udp.read(incomingPacket, 255);

        if (len > 0) {
            incomingPacket[len] = 0;
        }

        sscanf(incomingPacket, "%f,%f,%f,%d", &errorX, &errorY, &targetArea, &targetId);
        if (abs(errorX) < 15) errorX = 0;
        if (abs(errorY) < 15) errorY = 0;

        // Cáculo da variação de tempo (dt)
        unsigned long currentTime = millis();
        float dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;
        if (dt <= 0) return;

        // --- Controle PID ---
        integralX += errorX * dt;
        float derivativeX = (errorX - previousErrorX) / dt;
        float outputX =
            KpX * errorX +
            KiX * integralX +
            KdX * derivativeX;
        previousErrorX = errorX;

        integralY += errorY * dt;
        float derivativeY = (errorY - previousErrorY) / dt;
        float outputY =
            KpY * errorY +
            KiY * integralY +
            KdY * derivativeY;
        previousErrorY = errorY;

        // --- MOVE SERVOS ---
        posX -= outputX;
        posY += outputY;
        posX = constrain(posX, 10, 170);
        posY = constrain(posY, 30, 150);
        servoX.write(posX);
        servoY.write(posY);

        // ----------------------------
        // LÓGICA DA BASE
        // ----------------------------

        // --- Limites de saturação do servo X (considerando a folga de 3°) ---
        bool servoTravouDireita = posX >= (SERVO_RIGHT_LIMIT - 3);
        bool servoTravouEsquerda = posX <= (SERVO_LEFT_LIMIT + 3);

        // --- Quanto o servo já está "torcido" para um lado (0 = centro) ---
        // Usado para decidir se a base deve girar mesmo sem estar 100% saturada,
        // e também para saber quando o pan-tilt está "olhando de lado" o
        // suficiente para que avançar/recuar não faça sentido.
        const float SERVO_CENTRO = 90.0;
        float desvioServoX = posX - SERVO_CENTRO;          // + = girado p/ direita, - = p/ esquerda
        const float DESVIO_MAX_PARA_AVANCAR = 25.0;        // graus de tolerância

        bool alvoCentralizadoNoEixoX = abs(desvioServoX) < DESVIO_MAX_PARA_AVANCAR;

        // --- Ganho de correção diferencial entre as rodas ---
        // erroX > 0  => alvo à direita do centro da imagem => puxa a base p/ direita
        // erroX < 0  => alvo à esquerda => puxa a base p/ esquerda
        const float GANHO_CORRECAO_DIFERENCIAL = 0.15;

        enum EstadoBase { GIRAR_DIREITA, GIRAR_ESQUERDA, AVANCAR, RECUAR, PARADO };
        EstadoBase estado = PARADO;

        // 1) Prioridade máxima: servo realmente saturado (girar a base para
        //    "liberar" o servo e ele voltar a poder seguir o alvo).
        if (servoTravouDireita && errorX < -20) {
            estado = GIRAR_DIREITA;
        }
        else if (servoTravouEsquerda && errorX > 20) {
            estado = GIRAR_ESQUERDA;
        }
        // 2) Servo não saturado, mas girado demais para o lado: ainda não vale
        //    avançar/recuar, melhor terminar de girar a base para alinhar.
        else if (!alvoCentralizadoNoEixoX) {
            estado = (desvioServoX > 0) ? GIRAR_DIREITA : GIRAR_ESQUERDA;
        }
        // 3) Servo centralizado: agora sim, avaliar avanço/recuo pela área.
        else {
            float areaError = AREA_SETPOINT - targetArea;
            if (abs(areaError) < AREA_DEADBAND) {
                estado = PARADO;
            }
            else if (areaError > 0) {
                estado = AVANCAR;
            }
            else {
                estado = RECUAR;
            }
        }

        // --- Executa o estado escolhido (uma única chamada de motor por ciclo) ---
        switch (estado) {

            case GIRAR_DIREITA: {
                int pwm = abs(errorX) * 0.2;
                pwm = constrain(pwm, 50, 120);
                giraDireita(pwm);

                // Recentraliza o servo enquanto a base gira
                posX -= abs(errorX) * 0.03;
                posX = constrain(posX, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                servoX.write(posX);
                break;
            }

            case GIRAR_ESQUERDA: {
                int pwm = abs(errorX) * 0.2;
                pwm = constrain(pwm, 50, 120);
                giraEsquerda(pwm);

                posX += abs(errorX) * 0.03;
                posX = constrain(posX, SERVO_LEFT_LIMIT, SERVO_RIGHT_LIMIT);
                servoX.write(posX);
                break;
            }

            case AVANCAR: {
                float areaError = AREA_SETPOINT - targetArea;
                int pwmBase = abs(areaError) * 0.2;
                pwmBase = constrain(pwmBase, 50, 120);

                // Correção diferencial: se o alvo ainda está um pouco fora do
                // centro (mesmo dentro da tolerância), uma roda recebe mais
                // PWM que a outra para curvar suavemente em direção ao alvo
                // enquants avança.
                int correcao = errorX * GANHO_CORRECAO_DIFERENCIAL;
                int pwmEsq = constrain(pwmBase + correcao, 0, 150);
                int pwmDir = constrain(pwmBase - correcao, 0, 150);

                // Motor 1 = lado esquerdo, Motor 2 = lado direito (ajuste se
                // a fiação física for invertida na sua base)
                setMotorPWM(1, pwmEsq, 0);
                setMotorPWM(2, pwmDir, 0);
                break;
            }

            case RECUAR: {
                float areaError = AREA_SETPOINT - targetArea;
                int pwmBase = abs(areaError) * 0.2;
                pwmBase = constrain(pwmBase, 50, 120);

                int correcao = errorX * GANHO_CORRECAO_DIFERENCIAL;
                // Ao recuar, a correção de direção é invertida (a base anda de
                // ré, então "puxar a frente para a direita" significa inverter
                // qual roda recebe mais força)
                int pwmEsq = constrain(pwmBase - correcao, 0, 150);
                int pwmDir = constrain(pwmBase + correcao, 0, 150);

                setMotorPWM(1, 0, pwmEsq);
                setMotorPWM(2, 0, pwmDir);
                break;
            }

            case PARADO:
            default:
                pararMotores();
                break;
        }
    }
}