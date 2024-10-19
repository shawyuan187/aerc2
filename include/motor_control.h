#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// 連接全域變數，用來在不同的檔案中共享變數

// 左右輪的馬達和編碼器引腳
// extern功能，將變數定義在其他檔案中
// const功能，將變數設定為唯讀
extern const int encoderLeftPinA;  // 左輪編碼器 A 相
extern const int encoderRightPinA; // 右輪編碼器 A 相
extern const int motorLeftPWM;     // 左輪馬達 PWM 控制
extern const int motorRightPWM;    // 右輪馬達 PWM 控制
extern const int motorLeftDir;     // 左輪馬達方向
extern const int motorRightDir;    // 右輪馬達方向
extern const int IR[5];            // A1~A5為紅外線數值

extern volatile int IR_LL; // 最左的紅外線感測器
extern volatile int IR_L;  // 中左的紅外線感測器
extern volatile int IR_M;  // 中間的紅外線感測器
extern volatile int IR_R;  // 中右的紅外線感測器
extern volatile int IR_RR; // 最右的紅外線感測器

extern volatile long pulseLeft;  // 左輪的脈衝數
extern volatile long pulseRight; // 右輪的脈衝數
// 指令宣告區，用來快速查看指令的功能，案住Ctrl並點擊名稱即可跳轉到指令的功能

// 更新IR感測器, 白色為0, 黑色為1
void IR_update();
// 更新左輪的脈衝數
void updateLeftPulse();
// 更新右輪的脈衝數
void updateRightPulse();
// 控制左右輪的馬達
void motor(int speedL, int speedR);
// 控制左右輪的馬達，並移動到指定的目標脈衝數
void controlMotors(int speedL, int speedR, long targetPulses, bool autoSync);
// PID循跡
void PID_trail(bool useFiveIR, bool (*exitCondition)(), float Kp = 0, float Kd = 0, float Ki = 0, int baseSpeed = 250);
// 循跡
void trail();
// 小左
void small_turn_left();
// 小右
void small_turn_right();
// 中左
void mid_turn_left();
// 中右
void mid_turn_right();
// 大左
void big_turn_left();
// 大右
void big_turn_right();
// 前進
void forward();
// 停止
void stop();
// 執行某個指令一段時間ms
void cmd_for_ms(void (*command)(), unsigned long ms);
void trail_X();
#endif // MOTOR_CONTROL_H