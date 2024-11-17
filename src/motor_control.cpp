#include <Arduino.h>
#include <U8g2lib.h>
#include "motor_control.h"

// 使用 U8g2 庫並使用 SoftwareWire 作為 I2C 通訊
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/10, /* data=*/9, /* reset=*/U8X8_PIN_NONE);

#define IR_OFFSET 450

// 更新IR感測器, 白色為0, 黑色為1
void IR_update()
{
    // 讀取IR感測器，白色為0，黑色為1
    IR_LL = analogRead(IR[0]) > IR_OFFSET ? 1 : 0;
    IR_L = analogRead(IR[1]) > IR_OFFSET ? 1 : 0;
    IR_M = analogRead(IR[2]) > IR_OFFSET ? 1 : 0;
    IR_R = analogRead(IR[3]) > IR_OFFSET ? 1 : 0;
    IR_RR = analogRead(IR[4]) > IR_OFFSET ? 1 : 0;
}

void motor(int speedL, int speedR)
{
    digitalWrite(motorLeftDir, speedL < 0 ? LOW : HIGH);
    digitalWrite(motorRightDir, speedR < 0 ? LOW : HIGH);
    analogWrite(motorLeftPWM, abs(speedL));
    analogWrite(motorRightPWM, abs(speedR));
}

// 沿著黑線走，紅外線感測器的數值為0~1023，白色為0，黑色為1023
// 排列方式為IR[0]~IR[4]，IR[0]為最左邊的感測器，IR[4]為最右邊的感測器
// 讀取IR感測器，白色為0，黑色為1

void PID_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms, bool useUltraSonic)
{
    const int minimumSpeed = -255; // 最小速度
    const int maximumSpeed = 255;  // 最大速度
    int lastError = 0;             // 上一次的偏差值
    int integral = 0;              // 積分項

    unsigned long start_time = millis();

    while (true)
    {
        if (ms > 0 && millis() - start_time >= ms)
        {
            break;
        }

        IR_update();

        if (useUltraSonic)
        {
            ultrasonic();
        }
        // 計算偏差值
        int error = 0;

        if (useFiveIR)
        {
            if (IR_LL == 0 && IR_L == 0 && IR_M == 1 && IR_R == 0 && IR_RR == 0)
            {
                error = 0;
            }
            else if (IR_LL == 0 && IR_L == 1 && IR_M == 1 && IR_R == 0 && IR_RR == 0)
            {
                error = -0.4;
            }
            else if (IR_LL == 0 && IR_L == 0 && IR_M == 1 && IR_R == 1 && IR_RR == 0)
            {
                error = 0.4;
            }
            else if (IR_LL == 0 && IR_L == 1 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -1.9;
            }
            else if (IR_LL == 0 && IR_L == 0 && IR_M == 0 && IR_R == 1 && IR_RR == 0)
            {
                error = 1.9;
            }
            else if (IR_LL == 1 && IR_L == 1 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -2.8;
            }
            else if (IR_LL == 0 && IR_L == 0 && IR_M == 0 && IR_R == 1 && IR_RR == 1)
            {
                error = 2.8;
            }
            else if (IR_LL == 1 && IR_L == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -4.4;
            }
            else if (IR_LL == 0 && IR_L == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 1)
            {
                error = 4.4;
            }
            else
            {
                error = lastError;
            }
        }
        else
        {
            if (IR_L == 0 && IR_M == 1 && IR_R == 0)
            {
                error = 0;
            }
            else if (IR_L == 1 && IR_M == 1 && IR_R == 0)
            {
                error = -0.4;
            }
            else if (IR_L == 0 && IR_M == 1 && IR_R == 1)
            {
                error = 0.4;
            }
            else if (IR_L == 1 && IR_M == 0 && IR_R == 0)
            {
                error = -1.9;
            }
            else if (IR_L == 0 && IR_M == 0 && IR_R == 1)
            {
                error = 1.9;
            }
            else
            {
                error = lastError;
            }
        }

        // 計算積分項
        integral += error;

        // 計算微分項
        int derivative = error - lastError;

        // 計算調整值
        int adjustment = Kp * error + Ki * integral + Kd * derivative;

        // 計算新的馬達速度
        int speedL = baseSpeed + adjustment;
        int speedR = baseSpeed - adjustment;

        // 限制速度在最小和最大速度之間
        speedL = constrain(speedL, minimumSpeed, maximumSpeed);
        speedR = constrain(speedR, minimumSpeed, maximumSpeed);

        // 設置馬達速度
        motor(speedL, speedR);

        // 更新上一次的偏差值
        lastError = error;

        if (ms == 0 && exitCondition())
        {
            break;
        }
    }
}

void trail()
{
    IR_update();
    if (IR_M)
    {
        if (IR_L)
        {
            small_turn_left();
        }
        else if (IR_R)
        {
            small_turn_right();
        }
        else
        {
            forward();
        }
    }
    else
    {
        if (IR_L || IR_LL)
        {
            if (IR_LL)
            {
                big_turn_left();
            }
            else
            {
                mid_turn_left();
            }
        }
        else if (IR_R || IR_RR)
        {
            if (IR_RR)
            {
                big_turn_right();
            }
            else
            {
                mid_turn_right();
            }
        }
    }
}

void forward()
{
    IR_update();
    motor(255, 255);
}

void small_turn_left()
{
    IR_update();
    motor(220, 255);
}

void small_turn_right()
{
    IR_update();
    motor(255, 220);
}

void mid_turn_left()
{
    IR_update();
    motor(100, 255);
}

void mid_turn_right()
{
    IR_update();
    motor(255, 100);
}

void big_turn_left()
{
    IR_update();
    motor(30, 245);
}

void big_turn_right()
{
    IR_update();
    motor(245, 30);
}

void stop()
{
    IR_update();
    motor(-255, -255);
    delay(10);
    motor(0, 0);
}

void cmd_for_ms(void (*command)(), unsigned long ms)
{
    unsigned long start_time = millis();
    while (millis() - start_time < ms)
    {
        command();
    }
    stop();
}

void slow_trail()
{
    IR_update();
    if (IR_M)
    {
        if (IR_L)
        {
            motor(110, 128);
        }
        else if (IR_R)
        {
            motor(128, 110);
        }
        else
        {
            motor(128, 128);
        }
    }
    else
    {
        if (IR_L || IR_LL)
        {
            if (IR_LL)
            {
                motor(15, 123);
            }
            else
            {
                motor(50, 128);
            }
        }
        else if (IR_R || IR_RR)
        {
            if (IR_RR)
            {
                motor(123, 15);
            }
            else
            {
                motor(128, 50);
            }
        }
    }
}

void OLED_init()
{
    u8g2.begin();
    u8g2.setContrast(255); // 增加此行以設定最大亮度
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "OLED Init OK!"); // 增加初始化確認訊息
    u8g2.sendBuffer();
    delay(1000);
}

void OLED_display()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf); // 改用較小的字型以確保顯示空間足夠

    char buffer[32]; // 用於格式化字串

    // 使用 sprintf 格式化數值，避免直接使用 print
    sprintf(buffer, "IR_LL:%4d", analogRead(IR[0]));
    u8g2.drawStr(0, 10, buffer);

    sprintf(buffer, "IR_L:%4d", analogRead(IR[1]));
    u8g2.drawStr(0, 20, buffer);

    sprintf(buffer, "IR_M:%4d", analogRead(IR[2]));
    u8g2.drawStr(0, 30, buffer);

    sprintf(buffer, "IR_R:%4d", analogRead(IR[3]));
    u8g2.drawStr(0, 40, buffer);

    sprintf(buffer, "IR_RR:%4d", analogRead(IR[4]));
    u8g2.drawStr(0, 50, buffer);

    u8g2.sendBuffer();
    delay(100);
}

void PID_trail_left(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms)
{
    const int minimumSpeed = -255; // 最小速度
    const int maximumSpeed = 255;  // 最大速度
    int lastError = 0;             // 上一次的偏差值
    int integral = 0;              // 積分項

    unsigned long start_time = millis();

    while (true)
    {
        if (ms > 0 && millis() - start_time >= ms)
        {
            break;
        }

        IR_update();
        // 計算偏差值
        int error = 0;

        if (useFiveIR)
        {
            if (IR_LL == 0 && IR_L == 0 && IR_M == 1 && IR_R == 0 && IR_RR == 0)
            {
                // error = 0;
                error = -0.4;
            }
            else if (IR_LL == 0 && IR_L == 1 && IR_M == 1 && IR_R == 0 && IR_RR == 0)
            {
                error = -0.4;
            }
            else if (IR_LL == 0 && IR_L == 1 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -1.9;
            }
            else if (IR_LL == 1 && IR_L == 1 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -2.8;
            }
            else if (IR_LL == 1 && IR_L == 0 && IR_M == 0 && IR_R == 0 && IR_RR == 0)
            {
                error = -4.4;
            }
            else
            {
                error = lastError;
            }
        }
        else
        {
            if (IR_L == 0 && IR_M == 1 && IR_R == 0)
            {
                // error = 0;
                error = -0.4;
            }
            else if (IR_L == 1 && IR_M == 1 && IR_R == 0)
            {
                error = -0.4;
            }
            else if (IR_L == 1 && IR_M == 0 && IR_R == 0)
            {
                error = -1.9;
            }
            else
            {
                error = lastError;
            }
        }

        // 計算積分項
        integral += error;

        // 計算微分項
        int derivative = error - lastError;

        // 計算調整值
        int adjustment = Kp * error + Ki * integral + Kd * derivative;

        // 計算新的馬達速度
        int speedL = baseSpeed + adjustment;
        int speedR = baseSpeed - adjustment;

        // 限制速度在最小和最大速度之間
        speedL = constrain(speedL, minimumSpeed, maximumSpeed);
        speedR = constrain(speedR, minimumSpeed, maximumSpeed);

        // 設置馬達速度
        motor(speedL, speedR);

        // 更新上一次的偏差值
        lastError = error;

        if (ms == 0 && exitCondition())
        {
            break;
        }
    }
}

void ultrasonic()
{
    // 發送超音波觸發信號
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

void echoISR()
{
    if (digitalRead(echoPin) == HIGH)
    {
        // 記錄回波信號開始時間
        echoStart = micros();
    }
    else
    {
        // 記錄回波信號結束時間
        echoEnd = micros();
        // 計算持續時間
        unsigned long duration = echoEnd - echoStart;
        // 計算距離
        distance = (duration / 2.0) / 29.1;
    }
}

void PID_right(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop)
{
    PID_trail(false, []()
              { return (IR_RR == 1); }, Kp, Kd, 0, baseSpeed, 0);
    while (!(IR_RR == 0))
    {
        motor(baseSpeed, baseSpeed);
        IR_update();
    }
    if (useStop)
    {
        stop();
    }
    while (!(IR_RR))
    {
        IR_update();
        motor(turnSpeedL, turnSpeedR);
    }
    while (!(IR_RR == 0))
    {
        IR_update();
        motor(turnSpeedL, turnSpeedR);
    }
}

void PID_left(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop)
{
    PID_trail(false, []()
              { return (IR_LL == 1); }, Kp, Kd, 0, baseSpeed, 0);
    while (!(IR_LL == 0))
    {
        motor(baseSpeed, baseSpeed);
        IR_update();
    }
    if (useStop)
    {
        stop();
    }
    while (!(IR_LL))
    {
        IR_update();
        motor(turnSpeedL, turnSpeedR);
    }
    while (!(IR_LL == 0))
    {
        IR_update();
        motor(turnSpeedL, turnSpeedR);
    }
}