#include <Arduino.h>
#include <SoftwareSerial.h>
#include "motor_control.h"

const int motorLeftPWM = 5;  // 左輪馬達 PWM 控制, 轉速0~255
const int motorRightPWM = 6; // 右輪馬達 PWM 控制, 轉速0~255
const int motorLeftDir = 4;  // 左輪馬達方向
const int motorRightDir = 7; // 右輪馬達方向
const int buttonPin = 8;     // 按鈕連接到 Pin 2
int buttonState = 0;         // 變量來儲存按鈕狀態

volatile int IR_LL = 0;
volatile int IR_L = 0;
volatile int IR_M = 0;
volatile int IR_R = 0;
volatile int IR_RR = 0;
volatile long pulseLeft = 0;  // 左輪的脈衝數
volatile long pulseRight = 0; // 右輪的脈衝數

// A1~A5為紅外線數值
const int IR[5] = {A1, A2, A3, A4, A5};

void setup()
{
    // 設置馬達和編碼器的引腳
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(motorLeftPWM, OUTPUT);
    pinMode(motorRightPWM, OUTPUT);
    pinMode(motorLeftDir, OUTPUT);
    pinMode(motorRightDir, OUTPUT);
    // 將IR設定為輸入
    for (int i = 0; i < 5; i++)
    {
        pinMode(IR[i], INPUT); // 數值為0~1023，白色為0，黑色為1023
    }

    OLED_init(); // OLED 初始化
    // Serial.begin(9600);
}

void loop()
{
    // 按按鈕
    buttonState = digitalRead(buttonPin);
    while (buttonState)
    {
        buttonState = digitalRead(buttonPin);
        OLED_display();
    }
    delay(1000);

    // PID_trail(true, []()
    //           { return (IR_RR == 1); }, 90, 80, 0, 250, 0);

    // while (IR_RR)
    //     IR_update();

    // IR_update();
    // while (!(IR_M))
    // {
    //     IR_update();
    //     motor(50, -100);
    // }

    // PID_trail(true, []()
    //           { return (IR_RR == 1); }, 90, 80, 0, 250, 0); // 90, 80, 0, 250, 0

    // while (IR_RR)
    //     IR_update();

    // while (!(IR_M))
    // {
    //     IR_update();
    //     motor(50, -100);
    // }

    // PID_trail(false, []()
    //           { return (false); }, 100, 0, 0, 100, 1100);
    PID_trail(false, []()
              { return (IR_RR == 1); }, 100, 0, 0, 100, 0);

    // 4停止L角落

    IR_update();
    while (!(IR_R == 0 && IR_RR == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_R))
    {
        IR_update();
        motor(100, -100);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 100, 0, 0, 100, 0);

    IR_update();
    while (!(IR_R == 0 && IR_RR == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_R))
    {
        IR_update();
        motor(100, -100);
    }

    motor(100, 100);
    delay(100); // 可能要調整
    // 6停止十字

    PID_trail(false, []()
              { return (IR_LL == 1); }, 100, 0, 0, 100, 0);

    IR_update();
    while (!(IR_L == 0 && IR_LL == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_L))
    {
        IR_update();
        motor(-100, 100);
    }

    PID_trail(false, []()
              { return (IR_LL == 1); }, 100, 0, 0, 100, 0);

    IR_update();
    while (!(IR_L == 0 && IR_LL == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_L))
    {
        IR_update();
        motor(-100, 100);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 100, 0, 0, 100, 0);

    IR_update();
    while (!(IR_R == 0 && IR_RR == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_R))
    {
        IR_update();
        motor(100, -100);
    }

    PID_trail(false, []()
              { return (IR_LL == 1); }, 100, 0, 0, 100, 0);

    IR_update();
    while (!(IR_L == 0 && IR_LL == 0))
    {
        motor(100, 100);
        IR_update();
    }

    IR_update();
    while (!(IR_L))
    {
        IR_update();
        motor(-100, 100);
    }
    PID_trail(false, []()
              { return (false); }, 100, 0, 0, 100, 1500);

    PID_trail(false, []()
              { return (IR_LL == 1); }, 100, 0, 0, 100, 0);

    while (!(IR_L == 0 && IR_LL == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0))
    {
        motor(100, 100);
        IR_update();
    }
    while (!(IR_L))
    {
        IR_update();
        motor(-100, 100);
    }
    // 11停止銳角

    stop();
}