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
// ew
unsigned long startTime = 0; // 新增：記錄開始時間
unsigned long lapTime = 0;   // 新增：記錄單圈時間

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
    startTime = millis(); // 新增：記錄開始時間

    // * /////////////////////////////////////B圖/////////////////////////////////////
    // ! ///////////////////////////////////// 電壓7.74~7.93(wee) /////////////////////////////////////
    // ! ///////////////////////////////////// 電壓7.79~7.77//////////////////////////////////////

    int error = 0;

    error = PID_trail(true, []()
                      { return false; }, 70, 100, 0, 250, 400); // 前往2的右轉圓弧
    error = PID_trail(true, []()
                      { return (IR_RR == 1); }, 70, 100, 0, 130, 0); // 前往2的右轉圓弧
    stop();
    error = PID_trail(true, []()
                      { return (IR_RR == 0); }, 30, 0, 0, 90, 0, false, error);
    error = PID_trail(false, []()
                      { return (IR_R == 1); }, 30, 0, 0, 90, 0, false, error); // 2 的右轉

    error = PID_trail(true, []()
                      { return (false); }, 70, 100, 0, 120, 300); // 前往3的左轉圓弧

    error = PID_trail(false, []()
                      { return (IR_LL == 1); }, 70, 100, 0, 90, 0); // 前往3的左轉圓弧
    stop();
    error = PID_trail(true, []()
                      { return (IR_LL == 0); }, 50, 0, 0, 80, 0, false, error); // 3的左轉
    error = PID_trail(false, []()
                      { return (IR_L == 1); }, 50, 0, 0, 70, 0, false, error); // 3的左轉

    PID_right(100, 100, -100, 30, 0, true); // 4的右轉
    PID_right(100, 90, -90, 30, 0, true);   // 4的右轉
    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 250, 300); // 4的右轉圓弧

    PID_left(100, -100, 100, 30, 0, true);  // 5虛線後左轉
    PID_left(100, -100, 100, 30, 0, true);  // 6的左轉
    PID_right(100, 100, -100, 30, 0, true); // 6往7的右轉
    IR_update();
    // while (!(IR_M == 1))
    // {
    //     IR_update();
    //     motor(90, -90);
    // }
    error = PID_trail(false, []()
                      { return (false); }, 50, 10, 0, 60, 500, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false); //(8)
    delay(100);
    // error = PID_trail(false, []()
    //                   { return (IR_RR == 0); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false, error); //(8)
    delay(100);

    // error = PID_trail(false, []()
    //                   { return (IR_RR == 0); }, 80, 0, 0, 250, 0, false, error); //(8)
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 80, 0, 0, 250, 0, false, error); //(8)
    delay(50);
    motor(0, 0);
    delay(20);
    // error = PID_trail(false, []()
    //                   { return (IR_RR == 0); }, 80, 0, 0, 180, 0, false, error); //(8)
    // todo:
    PID_left(100, -100, 100, 20, 2, true); // 8 ~ 9 的左直角
    PID_trail(false, []()
              { return (false); }, 30, 0, 0, 60, 300); // 向前衝到 RR 看到黑線 (9)
    PID_trail(true, []()
              { return (IR_RR == 1); }, 30, 0, 0, 130, 0); // 向前衝到 RR 看到黑線 (9)
    delay(10);
    PID_right(80, 100, -100, 20, 2, true);  // 9 ~ 10 的右角
    PID_left(100, -100, 100, 20, 2, true);  // 10 的左直角
    PID_right(100, 100, -100, 20, 2, true); // 10 ~ 11 的右角
    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 30, 0, 0, 100, 0, false);
    error = PID_trail(true, []()
                      { return (false); }, 30, 0, 0, 70, 500, false, error);
    error = PID_trail(true, []()
                      { return (IR_L == 0 && IR_M == 0 && IR_R == 0); }, 30, 0, 0, 60, 0, false, error); // 11 + 12 的弧形
    error = PID_trail(true, []()
                      { return (false); }, 30, 0, 0, 60, 700, false, error); // 在紅線前轉正

    // ! /////////////////////////////////////開始右避障循跡///////////////////////////////////////
    distance = 0;
    PID_trail(false, []()
              { return (distance > 0 && distance <= 15); }, 40, 0, 0, 120, 0, true);
    stop();
    while (!(IR_LL))
    {
        IR_update();
        motor(110, 10);
    }
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(100, 0);
    }
    delay(100);
    while (!(IR_L))
    {
        IR_update();
        motor(68, 170);
    }
    // ! /////////////////////////////////////結束右避障循跡///////////////////////////////////////
    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 10, 200);
    // ! /////////////////////////////////////開始左避障循跡///////////////////////////////////////
    distance = 0;
    PID_trail(false, []()
              { return (distance > 0 && distance <= 14); }, 40, 0, 0, 100, 0, true);
    stop();
    while (!(IR_RR))
    {
        IR_update();
        motor(0, 110);
    }
    while (!(IR_RR == 0))
    {
        IR_update();
        motor(0, 100);
    }
    delay(100);
    while (!(IR_R))
    {
        IR_update();
        motor(170, 65);
    }
    // ! /////////////////////////////////////結束左避障循跡/////////////////////////////////////
    PID_left(120, -120, 120, 20, 2, true); // 左轉正到 14 的線上
    PID_left(100, -100, 100, 30, 0, true); // 14 ~ 15 的左直角
    PID_left(100, -100, 80, 30, 0, true);  // 15 ~ 16 的左直角
    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 60, 500);
    PID_trail(true, []()
              { return (IR_M == 0 && IR_R == 0 && IR_L == 0); }, 30, 0, 0, 255, 0); // 向前衝到 16 的空白處
    PID_trail(true, []()
              { return (IR_M == 1 or IR_R == 1 or IR_L == 1); }, 30, 0, 0, 255, 0); // 衝到 16 的黑線
    delay(70);
    PID_right(100, 100, -100, 30, 0, true); // 16 ~ 17的右直角

    // TODO: trytrysee
    error = PID_trail(true, []()
                      { return (IR_RR == 1); }, 70, 100, 0, 150, 0);
    stop(); // 直走衝到 RR 看到黑線
    PID_trail(true, []()
              { return (IR_R == 1); }, 70, 100, 0, 90, 0, false, error); // 超出 17 的黑線
    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 70, 100, 0, 150, 0);
    stop(); // 完成 18 的 U 轉彎 + 直走到 LL 看到黑線
    error = PID_trail(true, []()
                      { return (IR_R == 1); }, 50, 20, 0, 90, 0, false, error);

    // TODO: trytrysee
    PID_right(100, 100, -100, 30, 0, true); // 19 ~ 20 的右直角
    error = PID_trail(true, []()
                      { return (false); }, 30, 0, 0, 50, 600);
    PID_trail(true, []()
              { return (IR_R == 1 && IR_M == 1 && IR_L == 1); }, 30, 0, 0, 255, 0, false, error); // 衝 FINISH
    
    // ! ///////////////////////////////////// 電壓7.74~7.93 /////////////////////////////////////

    motor(-250, -250); // 停止馬達
    delay(30);
    motor(0, 0); // 停止馬達
    delay(30);
    lapTime = millis() - startTime; // 新增：計算單圈時間
};