#include <Arduino.h>
#include "motor_control.h"

// volatile功能，告訴編譯器，這個變數可能會在程式的不同部分被更改
// 所以編譯器要保持從記憶體中讀取變數的最新值，而不是從暫存器中讀取

// 更新IR感測器, 白色為0, 黑色為1
void IR_update()
{
    // 讀取IR感測器，白色為0，黑色為1
    IR_LL = analogRead(IR[0]) > 450 ? 1 : 0;
    IR_L = analogRead(IR[1]) > 450 ? 1 : 0;
    IR_M = analogRead(IR[2]) > 450 ? 1 : 0;
    IR_R = analogRead(IR[3]) > 450 ? 1 : 0;
    IR_RR = analogRead(IR[4]) > 450 ? 1 : 0;
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
void PID_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed)
{
    const int minimumSpeed = -255; // 最小速度
    const int maximumSpeed = 255;  // 最大速度
    int lastError = 0;             // 上一次的偏差值
    int integral = 0;              // 積分項

    while (true)
    {
        IR_update();

        if (exitCondition())
        {
            break;
        }
        // 計算偏差值
        int error = IR_L * -1 + IR_M * 0 + IR_R * 1; // 預設使用三個紅外線感測器
        if (useFiveIR)
        {
            error = IR_LL * -4 + IR_L * -1 + IR_M * 0 + IR_R * 1 + IR_RR * 4;
        }
        if (IR_M == 1 && IR_L == 0 && IR_R == 0)
        {
            error = 0;
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
    }
    stop();
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

// 前進
void forward()
{
    motor(255, 255);
}

// 小左
void small_turn_left()
{
    motor(200, 255);
}

// 小右
void small_turn_right()
{
    motor(255, 200);
}

// 中左
void mid_turn_left()
{
    motor(0, 255);
}

// 中右
void mid_turn_right()
{
    motor(255, 0);
}

// 大左
void big_turn_left()
{
    motor(-150, 150);
}

// 大右
void big_turn_right()
{
    motor(150, -150);
}

// 停止
void stop()
{
    motor(0, 0);
}