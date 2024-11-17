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
volatile bool measuring = false;      // 超音波是否正在量測

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

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0);

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0);

    while (IR_L == 0)
    {
        IR_update();
        motor(-25, 165); //(1)
    }

    while (IR_L == 0)
    {
        IR_update();
        motor(-25, 165); //(1)
    }

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0);
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0);

    while (IR_LL == 0)
    {
        IR_update();
        motor(-25, 165); //(1)
    }

    while (IR_LL == 0)
    {
        IR_update();
        motor(-25, 165); //(1)
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(1)

    // while (!(IR_RR == 0))
    // {
    //     forward();
    // }

    while (IR_RR == 1)
    {
        IR_update();
        motor(165, -25); //(2)
    }

    while (IR_RR == 0)
    {
        IR_update();
        motor(165, -25); //(2)
    }
    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(3)
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(3)

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(3)
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(3)

    while (!(IR_RR == 0))
    {
        IR_update();
        motor(165, -25); //(4)
    }

    while (!(IR_RR))
    {
        IR_update();
        motor(165, -25); //(4)
    }

    while (!(IR_RR == 0))
    {
        IR_update();
        motor(165, -25); //(4)
    }

    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 100, 750); //(4)

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(5)
    PID_trail(false, []()
              { return (IR_RR == 0); }, 30, 0, 0, 100, 0); //(5)

    while (IR_RR == 1)
    {
        IR_update();
        motor(165, -25); //(6)
    }

    while (IR_RR == 0)
    {
        IR_update();
        motor(165, -25); //(6)
    }

    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 100, 700); //(6)
    IR_update();
    while (!(IR_R))
    {
        motor(-100, 100);
        IR_update();
    }

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(6)

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(6)

    PID_trail(true, []()
              { return (IR_LL == 1); }, 30, 0, 0, 80, 0); //(6)
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(6)

    IR_update();
    while (!(IR_LL))
    {
        motor(-165, 50);
        IR_update();
    }
    while (!(IR_LL == 0))
    {
        motor(-165, 50);
        IR_update();
    }

    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 100, 2000); //(7)

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(8)

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(8)

    IR_update();
    while (!(IR_LL))
    {
        motor(-25, 125);
        IR_update();
    }
    while (!(IR_LL == 0))
    {
        motor(-25, 125);
        IR_update();
    }

    PID_trail(false, []()
              { return (false); }, 80, 0, 0, 250, 600); //(9)
    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(9)
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(9)
    IR_update();
    while (!(IR_LL))
    {
        motor(-25, 165);
        IR_update();
    }
    while (!(IR_LL == 0))
    {
        motor(-25, 165);
        IR_update();
    }
    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0);

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0);

    while (!(IR_L))
    {
        IR_update();
        motor(-25, 125); //(11)
    }
    for (int i = 0; i < 2; i++)
    {
        PID_rightU();
        PID_trail(false, []()
                  { return (false); }, 30, 0, 0, 100, 300); //(11)
        PID_leftU();
        // PID_trail(false, []()
        //           { return (false); }, 30, 0, 0, 100, 300); //(11)
    }
    PID_rightU();

    PID_trail(false, []()
              { return (false); }, 30, 0, 0, 100, 300); //(21)

        stop();
}
