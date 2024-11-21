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

    // ! /////////////////////////////////////1-12 電壓7.75~7.77/////////////////////////////////////
    PID_right(100, 100, -100, 30, 0, true); // 1的右直角
    PID_right(100, 90, -90, 30, 0, true);   // 2的右角
    PID_left(100, -100, 100, 30, 0, true);  // 3的左直角
    forward();
    delay(100);

    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 100, 500); //(3↑)  1250
    PID_trail(false, []()
              { return (IR_L == 0 && IR_M == 0 && IR_R == 0); }, 40, 0, 0, 90, 0); //(3↑)  1250
    stop();
    delay(100);

    unsigned long startTime = millis();
    // todo: SUPER FUCKING IMPORTANT
    while (true)
    {
        IR_update();
        motor(60, -100);

        if ((millis() - startTime >= 250) || (IR_L || IR_M || IR_R))
        {
            break;
        }
    }
    // TODO:SUPER FUCKING IMPORTANT END
    //! sooooo fucking need to fix this
    PID_trail_left(false, []()
                   { return (false); }, 60, 90, 0, 100, 1250); //(4↑)  1250

    stop();
    //! sooooo fucking need to fix this until here
    IR_update();
    while (!(IR_LL == 1))
    {
        IR_update();
        motor(100, 0);
    }
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(100, 0);
    }
    while (!(IR_R == 1))
    {
        IR_update();
        motor(100, 0);
    }
    motor(100, 100);
    delay(100);
    PID_trail(false, []()
              { return (false); }, 40, 30, 0, 90, 400); // 5 慢速穩定直走
    PID_left(90, -100, 100, 40, 30, true);              // 6的左直角
    stop();
    IR_update();
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
    while (!(IR_RR))
    {
        IR_update();
        motor(100, 0);
    }
    while (!(IR_M))
    {
        IR_update();
        motor(100, 0);
    }
    PID_right(100, 100, -100, 30, 0, true); // 7的右直角
    PID_left(100, -100, 100, 30, 0, true);  // 7的左直角
    PID_left(100, -100, 100, 30, 0, true);  // 8的左直角
    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); // 9的忽略右路
    delay(100);
    PID_right(100, 100, -100, 30, 0, true); // 9的右直角
    PID_right(100, 100, -100, 30, 0, true); // 10的右直角
    PID_left(100, -100, 100, 30, 0, true);  // 12的左直角
    PID_left(100, -100, 100, 30, 0, true);  // 12的左直角
    // ! ///////////////////////////////////// 1-12 結束, 電壓7.74~7.77 /////////////////////////////////////
    PID_left(100, -100, 100, 30, 0, true);  // 13的左直角
    PID_right(100, 100, -100, 30, 0, true); // 14的右直角

    // ? /////////////////////////////////////測試開始/////////////////////////////////////
    PID_trail(true, []()
              { return (false); }, 100, 110, 0, 250, 3000); // 15-18
    // ? /////////////////////////////////////測試結束/////////////////////////////////////

    //     PID_trail(true, []()
    //               { return (false); }, 30, 0, 0, 100, 2000); //(16,17↑)

    //     PID_trail(false, []()
    //               { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(16,17↑)
    //     PID_trail(false, []()
    //               { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(16,17↑)

    //     while (!(IR_LL == 1))
    //     {
    //         IR_update();
    //         motor(-25, 120);
    //     }

    //     while (!(IR_LL == 0))
    //     {
    //         IR_update();
    //         motor(-25, 120);
    //     }

    //     PID_trail(false, []()
    //               { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(18↑)

    //     PID_trail(false, []()
    //               { return (IR_RR == 0 and IR_LL == 0); }, 30, 0, 0, 100, 0); //(18↑)

    //     while (!(IR_LL == 1))
    //     {
    //         IR_update();
    //         motor(-25, 120);
    //     }

    //     while (!(IR_LL == 0))
    //     {
    //         IR_update();
    //         motor(-25, 120);
    //     }

    //     PID_trail(false, []()
    //               { return (IR_L == 1 and IR_R == 1 and IR_M == 1); }, 30, 0, 0, 100, 0); //(19↑)
    stop();
}