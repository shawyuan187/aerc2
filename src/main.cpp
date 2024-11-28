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
int error = 0;
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

    // * /////////////////////////////////////A圖/////////////////////////////////////
    // ! ///////////////////////////////////// 電壓7.74~7.93(wee) /////////////////////////////////////
    // ! ///////////////////////////////////// 電壓7.79~7.77/////////////////////////////////////

    PID_trail(true, []()
              { return (false); }, 75, 0, 0, 250, 500); // 1的循跡
    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 40, 5, 0, 90, 0);
    error = PID_trail(false, []()
                      { return (IR_LL == 0); }, 30, 0, 0, 90, 0, false, error);
    error = PID_trail(false, []()
                      { return (IR_LL == 1); }, 30, 0, 0, 90, 0, false, error);
    stop();
    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 90, 300); // 2的循跡
    error = PID_trail(true, []()
                      { return (IR_RR == 1); }, 40, 5, 0, 90, 0);
    error = PID_trail(false, []()
                      { return (IR_RR == 0); }, 30, 0, 0, 90, 0, false, error);
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 30, 0, 0, 90, 0, false, error);
    stop();
    PID_trail(false, []()
              { return (false); }, 60, 0, 0, 150, 200); // 3的循跡
    PID_right(100, 100, 0, 30, 0, true);                // 3的弧線結尾
    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 90, 300); // 4的循跡
    stop();
    PID_right(100, 100, -100, 30, 0, true); // 4的右直角
    PID_right(100, 100, 0, 30, 0, true);    // 5的右直角
    PID_left(100, -100, 100);               // 6的左直角
    PID_left(100, -100, 100);               // 7的左直角
    PID_right(100, 100, -100);              // 8的右直角
    PID_left(100, -100, 100);               // 9的左直角
    PID_right(100, 100, -100);              // 10的右直角
    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 200, 300); // 10的循跡
    PID_right(100, 100, -100);                          // 10的右直角
    PID_left(100, -100, 100);                           // 11的左直角
    PID_left(100, -100, 100);                           // 11的銳角
    PID_left(100, -100, 100);                           // 11的左修正

    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 100, 500); // 12的循跡, 讓車子盡量直行

    PID_trail(false, []()
              { return (IR_LL == 0 && IR_L == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0); }, 40, 0, 0, 90, 0); // 12的循跡, 讓車子盡量直行

    IR_update();
    while (!(IR_LL || IR_L || IR_M || IR_R || IR_RR))
    {
        IR_update();
        motor(255, 255);
    }
    cmd_for_ms(trail, 300);
    PID_left(90, -100, 90, 50, 50, true); // 12的左轉
    // ! /////////////////////////////////////大U開始/////////////////////////////////////

    for (int i = 0; i < 2; i++)
    {
        error = PID_trail(true, []()
                          { return (false); }, 70, 100, 0, 250, 150, false, error);
        error = PID_trail(true, []()
                          { return (IR_RR == 1); }, 70, 100, 0, 100, 0, false, error);
        stop();
        error = PID_trail(true, []()
                          { return (IR_RR == 0); }, 20, 0, 0, 70, 0, false, error);
        error = PID_trail(false, []()
                          { return (IR_RR == 1); }, 20, 0, 0, 70, 0, false, error);
        stop();
        PID_trail(false, []()
                  { return (false); }, 40, 0, 0, 100, 500);

        error = PID_trail(true, []()
                          { return (false); }, 70, 100, 0, 250, 150, false, error);
        error = PID_trail(true, []()
                          { return (IR_LL == 1); }, 70, 100, 0, 100, 0, false, error);
        stop();
        error = PID_trail(true, []()
                          { return (IR_LL == 0); }, 20, 0, 0, 70, 0, false, error);
        error = PID_trail(false, []()
                          { return (IR_LL == 1); }, 20, 0, 0, 70, 0, false, error);
        stop();
        PID_trail(false, []()
                  { return (false); }, 40, 0, 0, 100, 500);
    }
    error = PID_trail(true, []()
                      { return (IR_RR == 1); }, 70, 100, 0, 130, 0);
    stop();
    error = PID_trail(true, []()
                      { return (IR_RR == 0); }, 20, 0, 0, 70, 0, false, error);
    error = PID_trail(false, []()
                      { return (IR_RR == 1); }, 20, 0, 0, 70, 0, false, error);
    stop();
    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 100, 500);

    error = PID_trail(true, []()
                      { return (IR_LL == 1); }, 70, 100, 0, 130, 0);
    stop();
    error = PID_trail(true, []()
                      { return (IR_LL == 0); }, 20, 0, 0, 70, 0, false, error);
    error = PID_trail(false, []()
                      { return (IR_LL == 1); }, 20, 0, 0, 70, 0, false, error);
    stop();
    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 100, 300);

    // ! /////////////////////////////////////大U結束/////////////////////////////////////
    // ! /////////////////////////////////////小U開始/////////////////////////////////////
    for (int i = 0; i < 2; i++)
    {
        PID_right(100, 100, -100, 30, 0, true);
        PID_right(100, 100, -100, 30, 0, true);
        error = PID_trail(true, []()
                          { return (IR_LL == 1); }, 70, 100, 0, 130, 0);
        stop();
        error = PID_trail(true, []()
                          { return (IR_LL == 0); }, 20, 0, 0, 70, 0, false, error);
        error = PID_trail(false, []()
                          { return (IR_L == 1); }, 20, 0, 0, 70, 0, false, error);
        stop();
        PID_trail(false, []()
                  { return (false); }, 40, 0, 0, 100, 300);
    }
    PID_trail(false, []()
              { return (IR_LL == 1); }, 70, 100, 0, 130, 0); // 29 黑線
    PID_trail(false, []()
              { return (IR_LL == 0); }, 70, 100, 0, 130, 0); // 衝出 29 黑線
    motor(-100, -100);                                       // stop
    delay(100);                                              // stop
    while (!(IR_LL))                                         // 定輪轉
    {
        IR_update();
        motor(0, 150);
    }
    // ! /////////////////////////////////////小U結束/////////////////////////////////////
    // ! /////////////////////////////////////開始避障循跡///////////////////////////////////////
    distance = 0;
    PID_trail(false, []()
              { return (distance > 0 && distance <= 15); }, 40, 0, 0, 80, 0, true);
    stop();
    while (!(IR_LL))
    {
        IR_update();
        motor(100, 0);
    }
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(100, 0);
    }
    delay(100);
    while (!(IR_M))
    {
        IR_update();
        motor(40, 100);
    }
    while (!(IR_R == 0))
    {
        IR_update();
        motor(40, 100);
    }
    delay(50);
    PID_right(100, 100, -100, 40, 0, true);
    stop();

    // ! /////////////////////////////////////結束避障循跡///////////////////////////////////////
    PID_right(100, 100, -100, 40, 0, true);
    PID_right(100, 100, -100, 40, 0, true);
    PID_trail(false, []()
              { return (IR_R && IR_M && IR_L); }, 100, 0, 0, 250, 0);
    motor(-100, -100);
    delay(100);
    // ! ///////////////////////////////////// 電壓7.74~7.93 /////////////////////////////////////
    stop();
    lapTime = millis() - startTime; // 新增：計算單圈時間
}