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

const int trigPin = 2;                // 超音波 trig 引腳
const int echoPin = 3;                // 超音波 echo 引腳
float distance = 0;                   // 超音波量測距離
volatile unsigned long echoStart = 0; // 超音波回波開始時間
volatile unsigned long echoEnd = 0;   // 超音波回波結束時間

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
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(A0, INPUT); // 設定A0為輸入

    // 將IR設定為輸入
    for (int i = 0; i < 5; i++)
    {
        pinMode(IR[i], INPUT); // 數值為0~1023，白色為0，黑色為1023
    }
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
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
        OLED_display(); // OLED 顯示IR真實數值
    }
    delay(1000);

    // * /////////////////////////////////////B圖/////////////////////////////////////
    // ! ///////////////////////////////////// 電壓7.74~7.93 /////////////////////////////////////
    int error = 0;
    error = PID_trail(true, []()
                      { return (IR_RR == 1); }, 70, 100, 0, 150, 0); // 前往2的右轉圓弧
    stop();
    error = PID_trail(true, []()
                      { return (IR_R == 1); }, 30, 0, 0, 90, 0, false, error); // 2 的右轉

    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 70, 100, 0, 150, 0); // 前往3的左轉圓弧
    stop();
    error = PID_trail(true, []()
                      { return (IR_LL == 0); }, 50, 20, 0, 90, 0, false, error); // 3的左轉
    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 50, 0, 0, 90, 0, false, error); // 3的左轉

    PID_right(100, 100, -100, 30, 0, true); // 4的右轉
    PID_right(100, 90, -90, 30, 0, true);   // 4的右轉
    PID_left(100, -100, 100, 30, 0, true);  // 5虛線後左轉
    PID_left(100, -100, 100, 30, 0, true);  // 6的左轉
    PID_right(100, 100, -100, 30, 0, true); // 6往7的右轉
    IR_update();
    while (!(IR_M == 1))
    {
        IR_update();
        motor(90, -90);
    }

    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 0); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 0); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 0); }, 80, 0, 0, 250, 0, false, error); //(8)

    PID_left(100, -25, 135, 30, 0, true); // 8往9的左轉
    // ? ///////////////////////////////////////測試中///////////////////////////////////////
    // PID_right(100, 125, -25, 30, 0);      //(6)
    // PID_left(100, -100, 100);
    // PID_trail(true, []()
    //           { return (IR_LL == 1); }, 30, 0, 0, 100, 0);

    // while (!(IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }
    // while (!(IR_L))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }
    // PID_trail(false, []()
    //           { return (IR_LL == 1); }, 30, 0, 0, 100, 0);
    // while (!(IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }
    // while (!(IR_L))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }
    // PID_trail(false, []()
    //           { return (false); }, 30, 0, 0, 100, 100);
    // stop();
    // // ! ///////////////////////////////////////開始避障///////////////////////////////////////
    // distance = 0;
    // PID_trail(false, []()
    //           { return (distance <= 20 && distance > 0); }, 30, 0, 0, 100, 0, true);
    // stop();
    // IR_update();
    // while (!(IR_LL))
    // {
    //     motor(100, -100);
    //     IR_update();
    // }

    // while (!(IR_LL == 0))
    // {
    //     motor(250, -250);
    //     IR_update();
    // }
    // while (!(IR_L))
    // {
    //     IR_update();
    //     motor(80, 190);
    // }
    // distance = 0;
    // PID_trail(false, []()
    //           { return (distance <= 20 && distance > 0); }, 30, 0, 0, 50, 0, true);

    // stop();
    // IR_update();
    // while (!(IR_RR))
    // {
    //     motor(-100, 100);
    //     IR_update();
    // }

    // while (!(IR_RR == 0))
    // {
    //     motor(-250, 250);
    //     IR_update();
    // }
    // while (!(IR_R))
    // {
    //     IR_update();
    //     motor(205, 100);
    // }
    // PID_left(100, -100, 100);
    // // ! ///////////////////////////////////////結束避障///////////////////////////////////////
    // PID_left(100, -100, 100);
    // PID_left(100, -100, 100);
    // PID_trail(false, []()
    //           { return (false); }, 30, 0, 0, 100, 400);
    // PID_trail(false, []()
    //           { return (IR_RR == 0 && IR_R == 0 && IR_M == 0 && IR_L == 0 && IR_LL == 0); }, 30, 0, 0, 100, 0);
    // while (!(IR_RR || IR_R || IR_M || IR_L || IR_LL))
    // {
    //     motor(255, 255);
    //     IR_update();
    // }
    // PID_right(100, 100, -100, 30, 0, true);
    // PID_trail(true, []()
    //           { return (IR_LL == 1); }, 80, 90, 0, 250, 0);
    // while (!(IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }
    // while (!(IR_LL))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }
    // while (!(IR_LL == 0))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }

    // PID_trail(true, []()
    //           { return (false); }, 30, 0, 0, 100, 800);
    // PID_trail(false, []()
    //           { return (IR_R == 1 and IR_M == 1 and IR_L == 1); }, 80, 90, 0, 250, 0);
    // ! ///////////////////////////////////// 電壓7.74~7.93 /////////////////////////////////////
    stop();
}