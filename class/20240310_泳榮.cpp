#include <Arduino.h>
#include <Servo.h>
#include <Pixetto.h>

/*################################變數宣告區################################*/
// IR sansor A1~A5
int IR[5] = {A1, A2, A3, A4, A5};
// 速度控制:左邊馬達為PWM5，右邊馬達為PWM6
int mL = 5;
int mR = 6;
// 方向控制:左邊馬達為4，右邊馬達為7
int sL = 4;
int sR = 7;
Servo claw;              // 爪子伺服馬達
Servo arm;               // 手臂伺服馬達
Pixetto pxtSerial(5, 6); // 鏡頭辨識 RX, TX = { 5, 6 }
/*################################函數宣告區################################*/
// 馬達控制
void motor(int, int);
// 循跡
void trail();
// 小轉彎_左
void small_turn_left();
// 小轉彎_右
void small_turn_right();
// 中轉彎_左
void mid_turn_left();
// 中轉彎_右
void mid_turn_right();
// 大轉彎_左
void big_turn_left();
// 大轉彎_右
void big_turn_right();
// 前進
void forward();
// 停止
void stop();
// 後退
void back();
// 回到線用_左
void re_turn_left();
// 回到線_左
void return_to_line_left();
// 回到線用_右
void re_turn_right();
// 回到線_右
void return_to_line_right();
// 回到線_2
void return_to_line_2();
// 循跡直到路口
void trail_cross();
// 判斷是否為路口
bool cross();
// 初始化
void my_init();
// 夾起
void pick_up();
// 放下
void pick_down();
// 放下_2
void pick_down_2();
// B_區放下
void pick_down_B();
//  手臂上升
void arm_up();
// 手臂下降
void arm_down();
// 爪子打開
void claw_open();
// 爪子閉合
void claw_close();
// 循跡一段時間
void trail_for_ms(unsigned long ms);
// 搬運第一個貨物
void pick_first();
// 搬運第二個貨物
void pick_second();
// 搬運第三個貨物
void pick_third();
// 搬運第四個貨物
void pick_fourth();
// 循跡直到IR1~IR3都看到白色
void forward_to_white();
// 執行某個動作直到設定的某個IR先看到黑色再看到白色
void action_until_black_to_white(int IR, void (*action)());
// 執行某個動作直到IR1~IR3其中一個看到黑色
void action_until_black(void (*action)());
// 新的循跡測試
void trail_test();
// 左邊是目標物
void pick_A();
// 中間是目標物
void pick_B();
// 右邊是目標物
void pick_C();
// 偵測鏡頭辨識物體，輸入秒數，計算該秒數內偵測到最多的數值確定後回傳目標物的ID，-1為無偵測到物體，0為第一個物體，1為第二個物體，2為第三個物體
int detectObject(unsigned long ms);
/*################################程式初始化################################*/
void setup() // 程式初始化
{

    my_init();
    trail_cross();
    pick_C();
    // pick_up();
    // pick_first();
    // pick_second();
    // pick_third();
    // pick_fourth();
}

/*################################程式循環################################*/
void loop() // 程式循環
{
}

/*################################函數定義區################################*/
void pick_A() // A=左邊泡棉
{
    return_to_line_right();
    pick_down();
    while (!((analogRead(IR[1]) < 450) and (analogRead(IR[2]) < 450) and (analogRead(IR[3]) < 450)))
    {
        trail();
        if (analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450)
        {
            stop();
            delay(100);
        }
    }
    pick_up();
    return_to_line_right();
    trail_cross();
    return_to_line_right();
    trail_for_ms(1800);
    pick_down();
    back();
    delay(500);
    return_to_line_right();
    trail_cross();
    // /*###右邊結束回到路口面相白色的P###*/
    return_to_line_right();
    pick_down();
    while (!((analogRead(IR[1]) < 450) and (analogRead(IR[2]) < 450) and (analogRead(IR[3]) < 450)))
    {
        trail();
        if (analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450)
        {
            stop();
            delay(100);
        }
    }
    pick_up();
    return_to_line_right();
    trail_cross();
    return_to_line_right();
    trail_cross();
    forward();
    delay(500);
    stop();
    delay(100);
    trail_cross();
    // 直走直到IR1~IR3其中有一顆燈>450
    while (!(analogRead(IR[1]) > 450 or analogRead(IR[2]) > 450 or analogRead(IR[3]) > 450))
    {
        forward();
    }
    stop();
    delay(100);
    trail_cross();
    return_to_line_left();
    pick_down_B();
}

void pick_B()
{
    trail_for_ms(650);
    pick_up();
    back();
    delay(500);
    return_to_line_right();
    trail_cross();
    forward();
    delay(500);
    stop();
    delay(100);
    trail_cross();
    // 直走直到IR1~IR3其中有一顆燈>450
    while (!(analogRead(IR[1]) > 450 or analogRead(IR[2]) > 450 or analogRead(IR[3]) > 450))
    {
        forward();
    }
    stop();
    delay(100);
    trail_cross();
    return_to_line_left();
    pick_down_B();
    back();
    delay(200);
    re_turn_right();
    delay(100);
    return_to_line_right();
    trail_cross();
    return_to_line_right();
    trail_cross();
    trail_cross();
    forward();
    delay(500);
    trail_cross();
    return_to_line_left();
    pick_down();
    while (!((analogRead(IR[1]) < 450) and (analogRead(IR[2]) < 450) and (analogRead(IR[3]) < 450)))
    {
        trail();
        if (analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450)
        {
            stop();
            delay(100);
        }
    }
    pick_up();
    return_to_line_right();
    trail_cross();
}
void pick_C()
{
    return_to_line_left();
    pick_down();
    while (!((analogRead(IR[1]) < 450) and (analogRead(IR[2]) < 450) and (analogRead(IR[3]) < 450)))
    {
        trail();
        if (analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450)
        {
            stop();
            delay(100);
        }
    }
    pick_up();
    return_to_line_right();
    trail_cross();
    return_to_line_left();
    trail_for_ms(1800);
    pick_down();
    back();
    delay(500);
    return_to_line_right();
    trail_cross();
    // /*###左邊結束回到路口面相白色的P###*/
    return_to_line_left();
    pick_down();
    while (!((analogRead(IR[1]) < 450) and (analogRead(IR[2]) < 450) and (analogRead(IR[3]) < 450)))
    {
        trail();
        if (analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450)
        {
            stop();
            delay(100);
        }
    }
    pick_up();
    return_to_line_right();
    trail_cross();
    return_to_line_left();
    trail_cross();
    forward();
    delay(500);
    stop();
    delay(100);
    trail_cross();
    // 直走直到IR1~IR3其中有一顆燈>450
    while (!(analogRead(IR[1]) > 450 or analogRead(IR[2]) > 450 or analogRead(IR[3]) > 450))
    {
        forward();
    }
    stop();
    delay(100);
    trail_cross();
    return_to_line_left();
    pick_down_B();
}
void pick_first()
{
    trail_cross();
    forward_to_white();
    return_to_line_right();
    pick_down();
    trail_cross();
    pick_up();
    action_until_black_to_white(IR[0], back);
    return_to_line_left();
    trail_cross();
    action_until_black_to_white(IR[4], forward);
    action_until_black(re_turn_right);
    trail_cross();
    action_until_black_to_white(IR[0], forward);
    action_until_black(re_turn_left);
    trail_cross();
    pick_down_B();
}
void pick_second()
{
    back();
    delay(200);
    re_turn_right();
    delay(100);
    return_to_line_right();
    trail_cross();
    trail_cross();
    action_until_black_to_white(IR[4], forward);
    action_until_black(re_turn_right);
    trail_cross();
    forward_to_white();
    return_to_line_right();
    pick_down();
    trail_cross();
    pick_up();
    action_until_black_to_white(IR[4], back);
    return_to_line_right();
    trail_cross();
    action_until_black_to_white(IR[0], forward);
    action_until_black(re_turn_left);
    trail_cross();
    pick_down_B();
}
void pick_third()
{
    back();
    delay(200);
    re_turn_right();
    delay(100);
    return_to_line_right();
    trail_cross();
    trail_cross();
    forward_to_white();
    return_to_line_left();
    pick_down();
    trail_cross();
    pick_up();
    action_until_black_to_white(IR[0], back);
    return_to_line_left();
    trail_cross();
    pick_down_B();
}
void pick_fourth()
{
    back();
    delay(200);
    re_turn_right();
    delay(100);
    return_to_line_right();
    trail_cross();
    pick_down();
    re_turn_right();
    delay(300);
    // action_until_black_to_white(IR[4], re_turn_right);
    action_until_black(re_turn_right);
    trail_cross();
    pick_up();
    action_until_black_to_white(IR[0], back);
    return_to_line_left();
    trail_cross();
    action_until_black_to_white(IR[0], re_turn_left);
    pick_down_B();
}

void pick_down_B()
{
    trail_for_ms(1000);
    big_turn_right();
    delay(450);
    stop();
    forward();
    delay(300);
    stop();
    claw_open();
    delay(500);
}

void action_until_black(void (*action)())
{
    while (!(analogRead(IR[1]) > 450 or analogRead(IR[2]) > 450 or analogRead(IR[3]) > 450))
    {
        action();
    }
    stop();
}

void action_until_black_to_white(int IR, void (*action)())
{
    while (!(analogRead(IR) > 450))
    {
        action();
    }
    while (!(analogRead(IR) < 450))
    {
        action();
    }
    stop();
    delay(100);
}

void forward_to_white()
{
    while (!(analogRead(IR[1]) < 450 and analogRead(IR[2]) < 450 and analogRead(IR[3]) < 450))
    {
        forward();
    }
    stop();
    delay(100);
}

void trail_for_ms(unsigned long ms)
{
    unsigned long start_time = millis();
    while (millis() - start_time < ms)
    {
        trail();
    }
    stop();
}

void arm_up()
{
    arm.write(80); // 調整手臂角度範例
}

void arm_down()
{
    // arm.write(120); // 調整手臂角度範例
    for (int i = 0; i < 3; i++)
    {
        arm.write(100 + 10 * i);
        delay(100);
    }
}

void claw_open()
{
    claw.write(5);
}

void claw_close()
{
    claw.write(100);
}

void pick_up()
{
    claw_close();
    delay(100);
    arm_up();
}

void pick_down()
{
    arm_down();
    delay(100);
    claw_open();
    delay(100);
}

void pick_down_2()
{
    arm.write(120);
    delay(100);
    claw.write(20); // 調整爪子角度
}

void my_init()
{
    for (int i = 0; i < 5; i++)
    {
        pinMode(IR[i], INPUT);
    }

    pinMode(mL, OUTPUT);
    pinMode(mR, OUTPUT);
    pinMode(sL, OUTPUT);
    pinMode(sR, OUTPUT);

    claw.attach(9);
    arm.attach(10);
    pxtSerial.begin(); // 初始化Pixetto
}

void return_to_line_left()
{
    while (!(analogRead(IR[0]) > 450))
    {
        re_turn_left();
    }
    stop();
}

void return_to_line_right()
{
    while (!(analogRead(IR[4]) > 450))
    {
        re_turn_right();
    }
    stop();
}
void return_to_line_2()
{
    while (!(analogRead(IR[0]) > 450))
    {
        re_turn_left();
    }
    stop();
}

void trail_cross()
{
    while (!cross())
    {
        trail();
        // trail_test();
    }
    while (!(analogRead(IR[1]) < 450 or analogRead(IR[2]) < 450 or analogRead(IR[3]) < 450))
    {
        forward();
    }
    stop();
    delay(100);
}

void trail()
{
    if (analogRead(IR[2]) > 450)
    {
        if (analogRead(IR[1]) > 450)
        {
            mid_turn_left();
        }
        else if (analogRead(IR[3]) > 450)
        {
            mid_turn_right();
        }
        else if (analogRead(IR[1]) < 450 and analogRead(IR[3]) < 450)

        {
            forward();
        }
    }
    else
    {
        if (analogRead(IR[1]) > 450)
        {
            big_turn_left();
        }
        else if (analogRead(IR[3]) > 450)
        {
            big_turn_right();
        }
    }
}
void trail_test()
{
    if (analogRead(IR[2]) > 450)
    {
        if (analogRead(IR[1]) > 450)
        {
            // mid_turn_left();
            motor(0, 190);
        }
        else if (analogRead(IR[3]) > 450)
        {
            // mid_turn_right();
            motor(135, 0);
        }
        else if (analogRead(IR[1]) < 450 and analogRead(IR[3]) < 450)

        {
            // forward();
            motor(130, 130);
        }
    }
    else
    {
        if (analogRead(IR[1]) > 450)
        {
            // big_turn_left();
            motor(-190, 130);
        }
        else if (analogRead(IR[3]) > 450)
        {
            // big_turn_right();
            motor(105, -155);
        }
    }
}

bool cross()
{
    // 判斷IR1~IR3有沒有同時大於450，如果有的話就停止

    if (analogRead(IR[1]) > 450 and analogRead(IR[2]) > 450 and analogRead(IR[3]) > 450)
    {
        stop();
        return true;
    }
    else
    {
        return false;
    }
}

void forward()
{
    motor(100, 100);
}

void back()
{
    motor(-120, -120);
}
void small_turn_left()
{
    motor(80, 100);
}

void small_turn_right()
{
    motor(100, 80);
}

void mid_turn_left()
{
    motor(0, 110);
}
void mid_turn_right()
{
    motor(110, 0);
}

void big_turn_left()
{
    motor(-150, 100);
}

void big_turn_right()
{
    motor(90, -140);
}

void stop()
{
    motor(0, 0);
}

void re_turn_left()
{
    motor(-145, 100);
}
void re_turn_right()
{
    motor(110, -140);
}

void motor(int speedL, int speedR)
{
    // 判斷speedL跟speedR正負號決定方向
    digitalWrite(sL, speedL < 0 ? LOW : HIGH); // 左邊馬達方向設定
    digitalWrite(sR, speedR < 0 ? LOW : HIGH); // 右邊馬達方向設定

    analogWrite(mL, abs(speedL)); // 左邊馬達速度設定
    analogWrite(mR, abs(speedR)); // 右邊馬達速度設定
}

#define MAX_OBJECTS 3 // 物件 ID 的範圍為 0 到 2

int detectObject(unsigned long ms)
{
    // 初始化物件偵測次數的 array
    int objectCounts[MAX_OBJECTS] = {0};

    unsigned long start_time = millis();
    while (millis() - start_time < ms)
    {
        if (pxtSerial.isDetected())
        {
            // 增加對應物件的偵測次數
            int objectID = pxtSerial.getTypeID();
            if (objectID >= 0 && objectID < MAX_OBJECTS)
            {
                objectCounts[objectID]++;
            }
        }
    }

    // 找出偵測次數最多的物件
    int maxCount = 0;
    int maxObjectID = -1;
    for (int i = 0; i < MAX_OBJECTS; i++)
    {
        if (objectCounts[i] > maxCount)
        {
            maxCount = objectCounts[i];
            maxObjectID = i;
        }
        // Serial.print(objectCounts[i]);
        // Serial.print(" ");
    }

    // 回傳偵測次數最多的物件的 ID
    return maxObjectID;
}