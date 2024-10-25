## 主要檔案說明

### [include/motor_control.h](include/motor_control.h)

此檔案定義了馬達控制相關的全域變數和函數宣告，包括：

全域變數：

- `motorLeftPWM`：左輪馬達 PWM 控制
- `motorRightPWM`：右輪馬達 PWM 控制
- `motorLeftDir`：左輪馬達方向
- `motorRightDir`：右輪馬達方向
- `IR[5]`：紅外線感測器引腳
- `IR_LL`：最左的紅外線感測器值
- `IR_L`：中左的紅外線感測器值
- `IR_M`：中間的紅外線感測器值
- `IR_R`：中右的紅外線感測器值
- `IR_RR`：最右的紅外線感測器值
- `pulseLeft`：左輪的脈衝數
- `pulseRight`：右輪的脈衝數

函數宣告：

- `void IR_update()`：更新紅外線感測器的值
- `void updateLeftPulse()`：更新左輪的脈衝數
- `void updateRightPulse()`：更新右輪的脈衝數
- `void motor(int speedL, int speedR)`：控制左右輪的馬達
- `void controlMotors(int speedL, int speedR, long targetPulses, bool autoSync)`：控制馬達並移動到指定脈衝數
- `void PID_trail(bool useFiveIR, bool (*exitCondition)(), float Kp = 0, float Kd = 0, float Ki = 0, int baseSpeed = 250, unsigned long ms = 0)`：使用 PID 演算法進行循跡
- `void trail()`：循跡控制
- `void small_turn_left()`：小幅左轉
- `void small_turn_right()`：小幅右轉
- `void mid_turn_left()`：中等左轉
- `void mid_turn_right()`：中等右轉
- `void big_turn_left()`：大幅左轉
- `void big_turn_right()`：大幅右轉
- `void forward()`：前進
- `void stop()`：停止
- `void cmd_for_ms(void (*command)(), unsigned long ms)`：執行指定時間的指令
- `void trail_X()`：特定模式的循跡
- `void slow_trail()`：緩慢循跡
- `void OLED_init()`：初始化 OLED 顯示器
- `void OLED_display()`：OLED 顯示內容

### [src/main.cpp](src/main.cpp)

此檔案是專案的進入點，包含了初始化和主循環邏輯。主要功能包括：

- 初始化馬達、紅外線感測器、OLED 顯示器等硬體
- 等待按鈕觸發後開始執行循跡任務
- 根據紅外線感測器的數據，控制機器人的運動路徑
- 執行多個階段的循跡與轉向控制，實現指定的路徑導航

### [src/motor_control.cpp](src/motor_control.cpp)

此檔案實現了 `motor_control.h` 中宣告的所有函數。主要實作包括：

- 紅外線感測器數值的讀取與更新
- 馬達的控制與 PWM 調節
- 利用 PID 演算法進行精確的循跡控制
- 各種轉向動作的實現，如小幅轉向、中等轉向和大幅轉向
- OLED 顯示器的初始化與顯示功能

## 開發環境

- PlatformIO IDE
- Arduino 框架

## 硬體需求

- Arduino 主控板
- 馬達與馬達驅動模組
- 編碼器
- 紅外線感測器（共 5 個）
- OLED 顯示器
- 按鈕與相關配件

## 安裝與使用

1. 安裝 PlatformIO IDE。
2. 將此專案克隆到本地端。
3. 使用 PlatformIO IDE 開啟專案目錄。
4. 按照源碼註釋進行硬體連接，確保馬達、感測器和顯示器正確連接。
5. 編譯並將程式上傳到 Arduino 板。
6. 通電後，按下按鈕開始機器人循跡。

## 注意事項

- 確保所有硬體元件連接正確，特別是馬達方向和感測器引腳。
- 根據實際需要調整 PID 參數，以獲得最佳的循跡效果。
- 在調試過程中，可使用 OLED 顯示器查看感測器數據和狀態資訊。
