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
        OLED_display(); // OLED 顯示IR真實數值
    }
    delay(1000);

    // PID_trail(true, []()
    //           { return (IR_RR == 1); }, 90, 80, 0, 250, 0); // 1的循跡

    // while (IR_RR)
    //     IR_update();

    // IR_update();
    // while (!(IR_M))
    // {
    //     IR_update();
    //     motor(50, -100);
    // }

    // PID_trail(true, []()
    //           { return (IR_RR == 1); }, 90, 80, 0, 250, 0); // 2的循跡

    // while (IR_RR)
    //     IR_update();

    // while (!(IR_M))
    // {
    //     IR_update();
    //     motor(50, -100);
    // }

    // PID_trail(false, []()
    //           { return (false); }, 100, 0, 0, 100, 1100); // 3的循跡
    // PID_trail(false, []()
    //           { return (IR_RR == 1); }, 100, 0, 0, 100, 0); // 4的循跡

    // // 4停止

    // IR_update();
    // while (!(IR_R == 0 && IR_RR == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_R)) // 5的L右直角
    // {
    //     IR_update();
    //     motor(100, -100);
    // }

    // PID_trail(false, []()
    //           { return (IR_RR == 1); }, 100, 0, 0, 100, 0);

    // IR_update();
    // while (!(IR_R == 0 && IR_RR == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_R))
    // {
    //     IR_update();
    //     motor(100, -100);
    // }

    // motor(100, 100);
    // delay(100); // 可能要調整
    // // 6的十字要越過

    // PID_trail(false, []()
    //           { return (IR_LL == 1); }, 100, 0, 0, 100, 0);

    // IR_update();
    // while (!(IR_L == 0 && IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_L))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }

    // PID_trail(false, []()
    //           { return (IR_LL == 1); }, 100, 0, 0, 100, 0); // 7的循跡

    // IR_update();
    // while (!(IR_L == 0 && IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_L))
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }

    // PID_trail(false, []()
    //           { return (IR_RR == 1); }, 100, 0, 0, 100, 0); // 8的循跡

    // IR_update();
    // while (!(IR_R == 0 && IR_RR == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_R))
    // {
    //     IR_update();
    //     motor(100, -100);
    // }

    // PID_trail(false, []()
    //           { return (IR_LL == 1); }, 100, 0, 0, 100, 0); // 9的循跡

    // IR_update();
    // while (!(IR_L == 0 && IR_LL == 0))
    // {
    //     motor(100, 100);
    //     IR_update();
    // }

    // IR_update();
    // while (!(IR_L)) // 選左邊的岔路
    // {
    //     IR_update();
    //     motor(-100, 100);
    // }
    PID_trail(false, []()
              { return (false); }, 60, 60, 0, 100, 500); // 10的循跡

    PID_trail(false, []()
              { return (IR_RR == 1); }, 60, 60, 0, 100, 0);

    IR_update();
    while (!(IR_R == 0 && IR_RR == 0))
    {
        motor(100, 100);
        IR_update();
    }

    while (!(IR_RR))
    {
        IR_update();
        motor(100, -100);
    }

    PID_trail(false, []()
              { return (false); }, 60, 60, 0, 100, 500);

    PID_trail(false, []()
              { return (IR_LL == 1); }, 60, 60, 0, 100, 0); // 11的循跡

    while (!(IR_L == 0 && IR_LL == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0)) // 離開銳角黑線
    {
        motor(100, 100);
        IR_update();
    }
    while (!(IR_LL)) // 11的左銳角迴轉
    {
        IR_update();
        motor(-100, 100);
    }

    IR_update();
    while (IR_LL)
    {
        IR_update();
    }

    PID_trail(false, []()
              { return (false); }, 60, 60, 0, 100, 1000); // 11的循跡調時間, 經過小左轉
    PID_trail(false, []()
              { return (IR_LL == 0 && IR_L == 0 && IR_M == 1 && IR_R == 0 && IR_RR == 0); }, 60, 60, 0, 100, 0); // 12的循跡,讓車子進良維持直行

    IR_update();
    while (!(IR_L == 0 && IR_LL == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0)) // 直行後接著高速循跡到看到一片空白
    {
        trail();
    }

    IR_update();
    while (!(IR_LL || IR_L || IR_M || IR_R || IR_RR)) // 往前衝直到看到黑線
    {
        forward();
    }

    IR_update();
    while (!(IR_LL)) // 繼續高速循跡到左直角L
    {
        trail();
    }

    stop();
}