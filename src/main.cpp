#include <Arduino.h>
#include <SoftwareSerial.h>
#include "motor_control.h"

// 左右輪的馬達和編碼器引腳
const int encoderLeftPinA = 2;  // 左輪編碼器 A 相
const int encoderRightPinA = 3; // 右輪編碼器 A 相
const int motorLeftPWM = 5;     // 左輪馬達 PWM 控制, 轉速0~255
const int motorRightPWM = 6;    // 右輪馬達 PWM 控制, 轉速0~255
const int motorLeftDir = 4;     // 左輪馬達方向
const int motorRightDir = 7;    // 右輪馬達方向
const int buttonPin = 8;        // 按鈕連接到 Pin 2
int buttonState = 0;            // 變量來儲存按鈕狀態

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
    pinMode(encoderLeftPinA, INPUT);
    pinMode(encoderRightPinA, INPUT);
    pinMode(motorLeftPWM, OUTPUT);
    pinMode(motorRightPWM, OUTPUT);
    pinMode(motorLeftDir, OUTPUT);
    pinMode(motorRightDir, OUTPUT);
    // 將IR設定為輸入
    for (int i = 0; i < 5; i++)
    {
        pinMode(IR[i], INPUT); // 數值為0~1023，白色為0，黑色為1023
    }

    // 設置編碼器中斷
    // attachInterrupt(digitalPinToInterrupt(encoderLeftPinA), updateLeftPulse, RISING);
    // attachInterrupt(digitalPinToInterrupt(encoderRightPinA), updateRightPulse, RISING);

    // Serial.begin(9600);
}

void loop()
{
    // 按按鈕
    buttonState = digitalRead(buttonPin);
    while (buttonState)
    {
        buttonState = digitalRead(buttonPin);
    }
    delay(1000);

    // // 1 往前衝
    // IR_update();
    // while (!(IR_RR))
    // {
    //     trail();
    // }
    // // 1 超出前面
    // while (IR_RR)
    // {
    //     forward();
    // }

    // motor(-255, -255);
    // delay(50);
    // // 2 旋轉
    // IR_update();
    // while (!(IR_R))
    // {
    //     IR_update();
    //     motor(50, -100);
    // }
    // delay(10);
    // IR_update();
    // // 2 往左轉衝
    // while (!(IR_RR))
    // {
    //     mid_turn_left();
    // }
    // // 2 超出前面
    // while (IR_RR)
    // {
    //     IR_update();
    // }
    // // 3 往右旋轉
    // motor(255, -100);
    // delay(50);
    // motor(255, 70);
    // delay(300);
    // while (!(IR_R))
    // {
    //     IR_update();
    // }
    // 4 往前衝

    while (!(IR_RR))
    {
        trail();
    }

    // 4 超出前面
    while (IR_RR)
    {
        stop();
        IR_update();
    }

    // 5 向右原地旋轉
    while (!(IR_RR))
    {
        IR_update();
        motor(80, -90);
    }
    while (IR_RR)
    {
        IR_update();
    }

    // 5 往前衝
    while (!(IR_RR))
    {
        slow_trail();
    }
    while (IR_RR)
    {
        slow_trail();
    }
    delay(10);
    while (!(IR_RR))
    {
        IR_update();
        motor(128, -20);
    }
    while (IR_RR)
    {
        IR_update();
    }

    motor(128, 128);
    delay(100);
    while (!(IR_LL))
    {
        IR_update();
        motor(128, 128);
    }
    // while (!(IR_RR == 0 && IR_R == 0 && IR_M == 0 && IR_L == 0 && IR_LL == 0))
    // {
    //     IR_update();
    //     motor(128, 128);
    // }

    stop();
}