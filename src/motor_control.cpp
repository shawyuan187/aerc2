#include <Arduino.h>
#include <SoftwareWire.h>
#include <U8g2lib.h>
#include "motor_control.h"

// 建立 SoftwareWire 物件
SoftwareWire myWire(2, 3); // SDA 在 D2，SCL 在 D3

// 使用 U8g2 庫並使用 SoftwareWire 作為 I2C 通訊
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/3, /* data=*/2, /* reset=*/U8X8_PIN_NONE);

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

// 更新左輪的脈衝數
void updateLeftPulse()
{
    pulseLeft++;
}

// 更新右輪的脈衝數
void updateRightPulse()
{
    pulseRight++;
}

// 控制左右輪的馬達
void motor(int speedL, int speedR)
{
    digitalWrite(motorLeftDir, speedL < 0 ? LOW : HIGH);
    digitalWrite(motorRightDir, speedR < 0 ? LOW : HIGH);
    analogWrite(motorLeftPWM, abs(speedL));
    analogWrite(motorRightPWM, abs(speedR));
}

// 馬達脈衝對齊控制函數
void controlMotors(int initialSpeedL, int initialSpeedR, long targetPulses, bool autoSync)
{
    const int minimumSpeed = -255; // 最小速度
    const int maximumSpeed = 255;  // 最大速度
    float Kp = 2;

    int speedL = initialSpeedL;
    int speedR = initialSpeedR;

    pulseLeft = 0;
    pulseRight = 0;

    while (pulseLeft < targetPulses && pulseRight < targetPulses)
    {
        if (autoSync)
        {
            long pulseDifference = pulseLeft - pulseRight;

            int adjustment = Kp * pulseDifference;

            // 因為pulse永遠是正的但是馬達可能是負的(反轉)，所以要判斷正負來改變速度修正公式
            if (initialSpeedL >= 0)
                speedL = initialSpeedL - adjustment;
            else
                speedL = initialSpeedL + adjustment;

            if (initialSpeedR >= 0)
                speedR = initialSpeedR + adjustment;
            else
                speedR = initialSpeedR - adjustment;

            speedL = constrain(speedL, minimumSpeed, maximumSpeed);
            speedR = constrain(speedR, minimumSpeed, maximumSpeed);
        }
        motor(speedL, speedR);
    }

    motor(0, 0); // 停止馬達
    Serial.println("運行完成");
    Serial.print("Left Pulses: ");
    Serial.println(pulseLeft);
    Serial.print("Right Pulses: ");
    Serial.println(pulseRight);
    Serial.print("speedL: ");
    Serial.println(speedL);
    Serial.print("speedR: ");
    Serial.println(speedR);
    Serial.println("==================================");
}

// 沿著黑線走，紅外線感測器的數值為0~1023，白色為0，黑色為1023
// 排列方式為IR[0]~IR[4]，IR[0]為最左邊的感測器，IR[4]為最右邊的感測器
// 讀取IR感測器，白色為0，黑色為1

// PID循跡
void PID_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms)
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

// 循跡
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

void trail_X()
{
    IR_update();
    if (IR_M)
    {
        if (IR_L)
        {
            mid_turn_left();
        }
        else if (IR_R)
        {
            mid_turn_right();
        }
        else
        {
            forward();
        }
    }
    else
    {
        if (IR_L)
        {
            big_turn_left();
        }
        else if (IR_R)
        {
            big_turn_right();
        }
    }
}
// 前進
void forward()
{
    IR_update();
    motor(255, 255);
}

// 小左
void small_turn_left()
{
    IR_update();
    motor(220, 255);
}

// 小右
void small_turn_right()
{
    IR_update();
    motor(255, 220);
}

// 中左
void mid_turn_left()
{
    IR_update();
    motor(100, 255);
}

// 中右
void mid_turn_right()
{
    IR_update();
    motor(255, 100);
}

// 大左
void big_turn_left()
{
    IR_update();
    motor(30, 245);
}

// 大右
void big_turn_right()
{
    IR_update();
    motor(245, 30);
}

// 停止
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
    // 初始化 OLED 顯示器
    u8g2.begin();
}

void OLED_display()
{
    // 清除顯示器，準備更新顯示的內容
    u8g2.clearBuffer();

    // 設定顯示文字的位置
    u8g2.setFont(u8g2_font_ncenB08_tr);
    // 顯示各個變數的數值
    u8g2.setCursor(0, 10);
    u8g2.print("IR_LL: ");
    u8g2.print(analogRead(IR[0]));

    u8g2.setCursor(0, 20);
    u8g2.print("IR_L: ");
    u8g2.print(analogRead(IR[1]));

    u8g2.setCursor(0, 30);
    u8g2.print("IR_M: ");
    u8g2.print(analogRead(IR[2]));

    u8g2.setCursor(0, 40);
    u8g2.print("IR_R: ");
    u8g2.print(analogRead(IR[3]));

    u8g2.setCursor(0, 50);
    u8g2.print("IR_RR: ");
    u8g2.print(analogRead(IR[4]));

    // 顯示更新的內容到 OLED 上
    u8g2.sendBuffer();
    delay(100);
}
