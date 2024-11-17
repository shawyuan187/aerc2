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

    while (!(IR_RR)) // 直到RR紅外線看到黑
    {
        trail();
    }
    while (!(IR_RR == 0))
    {
        forward();
    }
    IR_update();
    while (!(IR_RR)) // 直到RR紅外線看到黑(1↑)
    {
        IR_update();
        motor(80, -110);
    }

    while (!(IR_RR == 0))
    {
        IR_update();
    }

    IR_update();
    while (!(IR_RR)) //(2↑)
    {
        trail();
    }

    while (IR_RR)
    {
        forward();
    }

    while (!(IR_RR))
    {
        IR_update();
        motor(70, -100);
    }
    while (!(IR_LL))
    {
        forward();
    }
    while (!(IR_LL == 0 && IR_L == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0))
    {
        forward();
    }
    while (!(IR_LL))
    {
        IR_update();
        motor(-100, 70); // 左迴轉
    }
    forward();
    delay(100);

    PID_trail(false, []()
              { return (false); }, 40, 0, 0, 100, 500); //(3↑)  1250
    PID_trail(false, []()
              { return (IR_L == 0 && IR_M == 0 && IR_R == 0); }, 40, 0, 0, 100, 0); //(3↑)  1250
    stop();

    unsigned long startTime = millis();
    // while (!(IR_M))
    while (true)
    {
        IR_update();
        motor(70, -100);

        if (millis() - startTime >= 300)
        {
            break;
        }
    }

    PID_trail_left(false, []()
                   { return (false); }, 50, 90, 0, 100, 1250); //(4↑)  1250

    stop();
    delay(50);

    forward();
    delay(100);

    motor(25, -165);
    delay(280);

    PID_trail(false, []()
              { return (false); }, 30, 0, 0, 100, 500);

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0);

    while (!(IR_RR == 1))
    {
        forward();
    }

    while (!(IR_RR == 0))
    {
        IR_update();
        motor(100, 100);
    }
    stop();
    delay(100);

    while (!(IR_RR == 1))
    {
        IR_update();
        motor(25, -165);
    }

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0);

    cmd_for_ms([]()
               { motor(-25, 165); }, 250);
    // 要改條件式 **********

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(8↑)

    IR_update();
    while (!(IR_LL == 0 && IR_L == 0))
    {
        IR_update();
        motor(100, 100);
    }
    IR_update();
    while (!(IR_LL))
    {
        IR_update();
        motor(-25, 165);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(9↑)

    PID_trail(false, []()
              { return (false); }, 30, 0, 0, 100, 100);

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0);

    IR_update();
    while (!(IR_RR == 0 && IR_R == 0))
    {
        IR_update();
        motor(100, 100);
    }
    IR_update();
    while (!(IR_RR))
    {
        IR_update();
        motor(110, -25);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(10↑)

    IR_update();
    while (!(IR_R == 0))
    {
        IR_update();
        motor(100, 100);
    }
    IR_update();
    while (!(IR_R))
    {
        IR_update();
        motor(120, -25);
    }

    PID_trail(true, []()
              { return (false); }, 20, 0, 0, 70, 650); //(12↑)

    PID_trail(false, []()
              { return (false); }, 10, 0, 0, 100, 1000);

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(13↑)

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0);

    IR_update();
    while (!(IR_LL))
    {
        IR_update();
        motor(-25, 165);
    }

    IR_update();
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(-25, 165);
    }
    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(13↑)

    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0);

    motor(100, 100);
    delay(50);
    IR_update();
    while (!(IR_LL))
    {
        IR_update();
        motor(-25, 165);
    }

    IR_update();
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(-25, 165);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(14↑)

    IR_update();
    while (!(IR_RR == 0))
    {
        IR_update();
        motor(100, 100);
    }
    IR_update();
    while (!(IR_LL == 1))
    {
        IR_update();
        motor(120, -25);
    }

    while (!(IR_LL == 0))
    {
        IR_update();
        motor(120, -25);
    }
    while (!(IR_RR))
    {
        IR_update();
        motor(120, -25);
    }
    while (!(IR_RR == 0))
    {
        IR_update();
        motor(120, -25);
    } //(15↑)

    PID_trail(true, []()
              { return (false); }, 30, 0, 0, 100, 2000); //(16,17↑)

    PID_trail(false, []()
              { return (IR_LL == 1); }, 30, 0, 0, 100, 0); //(16,17↑)
    PID_trail(false, []()
              { return (IR_LL == 0); }, 30, 0, 0, 100, 0); //(16,17↑)

    while (!(IR_LL == 1))
    {
        IR_update();
        motor(-25, 120);
    }

    while (!(IR_LL == 0))
    {
        IR_update();
        motor(-25, 120);
    }

    PID_trail(false, []()
              { return (IR_RR == 1); }, 30, 0, 0, 100, 0); //(18↑)

    PID_trail(false, []()
              { return (IR_RR == 0 and IR_LL == 0); }, 30, 0, 0, 100, 0); //(18↑)

    while (!(IR_LL == 1))
    {
        IR_update();
        motor(-25, 120);
    }

    while (!(IR_LL == 0))
    {
        IR_update();
        motor(-25, 120);
    }

    PID_trail(false, []()
              { return (IR_L == 1 and IR_R == 1 and IR_M == 1); }, 30, 0, 0, 100, 0); //(19↑)
    stop();
}
