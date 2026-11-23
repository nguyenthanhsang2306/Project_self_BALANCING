# HƯỚNG DẪN CẤU HÌNH & VẬN HÀNH XE TỰ CÂN BẰNG (STM32L476RG)

Tài liệu hướng dẫn toàn diện từ sơ đồ đấu nối phần cứng, cấu hình ngoại vi STM32CubeMX, giải thích chi tiết **tất cả các file trong dự án** và hướng dẫn sử dụng 2 dự án: **`Project_Self_PID`** (Tự cân bằng đứng yên bằng Cascade PID + Kalman) và **`Project_Motion`** (Vừa cân bằng vừa di chuyển qua ESP32-C6 WebSocket).

---

## Cấu trúc nhánh (Git Branches)

Repo sử dụng mô hình **1 nhánh khung nền (`main`) + 2 nhánh chức năng độc lập** ngay tại thư mục gốc (không bọc lồng thư mục con):

| Nhánh | Nội dung (tại thư mục gốc `/`) | Cách chuyển nhánh |
| :--- | :--- | :--- |
| **`main`** | **Bộ khung phần cứng & driver nền STM32L476RG**: Clock 80 MHz, `TIM1` PWM 10 kHz, `TIM2`/`TIM3` Encoder, `TIM6` 200 Hz, `I2C1` MPU6050 + Kalman, `motor.c`, `pid.c`, `USART2` | `git checkout main` |
| **`Project_self_PID`** | Firmware hoàn chỉnh **tự cân bằng đứng yên tại chỗ** (Cascade PID + Encoder giữ vị trí + `Project_Self_PID.ioc`) | `git checkout Project_self_PID` |
| **`Project_motion`** | Firmware hoàn chỉnh **cân bằng + điều khiển di chuyển `W/A/S/D`** (Velocity PI bumpless + `LPUART1` + `Project_Motion.ioc` + `ESP32_WiFi_Controller/`) | `git checkout Project_motion` |

---


## 1. Sơ đồ đấu nối dây phần cứng (Wiring Diagram)

> **Lưu ý an toàn:** Hãy tháo pin 18650 ra khỏi đế pin trong lúc cắm dây để tránh chạm chập gây hỏng linh kiện.

### Bước 1: Nguồn Pin cho TB6612FNG & STM32 (Chạy độc lập không cần cáp ST-LINK)
| Chân / Jumper | Nối đến đâu? | Vị trí trên STM32 Nucleo-L476RG |
| :--- | :--- | :--- |
| **VM** (TB6612) | Cực Dương (+) của Pin 2S 18650 (7.4V–8.4V) | Nối thẳng vào dây Đỏ (+) của đế Pin |
| **VIN** (STM32) | Cực Dương (+) của Pin 2S 18650 (7.4V–12V) | Lỗ **`VIN`** (cuối thanh nhựa đen **CN6 Power** bên trái, cạnh lỗ GND) |
| **Jumper `JP5`** | Chọn nguồn ngoài thay cho USB ST-LINK | Rút jumper từ `U5V` (chân 1–2) **cắm sang `E5V` (chân 2–3)** |
| **Jumper `JP1`** | Ngắt giới hạn dòng USB | **Tháo bỏ `JP1`** (hoặc cắm hở 1 chân) |
| **VCC** (TB6612) | Nguồn logic 3.3V từ STM32 | Lỗ **3V3** (thanh CN6 Power bên trái) |
| **GND** | Cực Âm (-) Pin & GND STM32 | Nối chung dây Đen (-) của Pin, chân **GND** của TB6612 và lỗ **GND** trên STM32 |
| **STBY** (TB6612) | Chân điều khiển bật Driver | Lỗ **A5** (tức chân `PC0` — cuối thanh CN8 bên trái) |

> **Lưu ý khi vừa chạy Pin vừa cắm cáp USB để xem `Motor_PID_Panel.exe`:**
> Khi `JP5` đang ở `E5V`, hãy **bật công tắc Pin trước** (đèn LED nguồn `LD3` sáng) $\rightarrow$ **sau đó mới cắm cáp USB vào máy tính**. Khi muốn quay lại dùng nguồn từ cáp USB ST-LINK mà không dùng Pin, chuyển `JP5` về lại `U5V` (chân 1–2).

---

### Bước 2: 6 dây tín hiệu điều khiển 2 bánh xe (Thanh nhựa đen bên phải STM32)
Đặt board STM32 Nucleo hướng cổng cáp USB lên trên. Nhìn vào dãy lỗ cắm nhựa màu đen bên phải (gồm thanh CN5 ở trên và CN9 ở dưới):

#### 1. Kênh A — Bánh 1 / Trái (3 dây):
* **PWMA** (TB6612) $\rightarrow$ Cắm vào lỗ **D7** (chân `PA8` / `TIM1_CH1`) — Lỗ trên cùng của thanh CN9.
* **AIN1** (TB6612) $\rightarrow$ Cắm vào lỗ **D5** (chân `PB4`) — Cách lỗ D7 một lỗ trống D6.
* **AIN2** (TB6612) $\rightarrow$ Cắm vào lỗ **D4** (chân `PB5`) — Nằm ngay sát dưới lỗ D5.

#### 2. Kênh B — Bánh 2 / Phải (3 dây):
* **PWMB** (TB6612) $\rightarrow$ Cắm vào lỗ **D8** (chân `PA9` / `TIM1_CH2`) — Lỗ dưới cùng của thanh CN5 (sát ngay trên lỗ D7).
* **BIN1** (TB6612) $\rightarrow$ Cắm vào lỗ **D10** (chân `PB6`) — Cách lỗ D8 một lỗ trống D9 về phía trên.
* **BIN2** (TB6612) $\rightarrow$ Cắm vào lỗ **D2** (chân `PA10`) — Nằm ở thanh CN9 dưới, cách lỗ D4 một lỗ trống D3.

> **Mẹo kiểm tra nhanh thứ tự các lỗ có cắm dây trên hàng bên phải (từ trên xuống dưới):**  
> `D10` (BIN1) $\rightarrow$ (trống D9) $\rightarrow$ `D8` (PWMB) $\rightarrow$ `D7` (PWMA) $\rightarrow$ `D6` (UART RX từ ESP32-C6 khi dùng `Project_Motion`, trống nếu chỉ chạy `Project_Self_PID`) $\rightarrow$ `D5` (AIN1) $\rightarrow$ `D4` (AIN2) $\rightarrow$ (trống D3) $\rightarrow$ `D2` (BIN2).

---

### Bước 3: Nối 2 động cơ GA12-N20 kèm Encoder (6 dây mỗi bánh)
Mỗi động cơ GA12-N20 có cụm 6 sợi dây gồm **2 dây cấp nguồn motor ở ngoài cùng** và **4 dây Encoder ở giữa**:

#### 1. Hai dây nguồn động cơ (nối vào TB6612FNG):
* **Động cơ Bánh Trái:**
  * Dây M1 (Đỏ) $\rightarrow$ Chân **AO1** trên TB6612.
  * Dây M2 (Trắng/Đen) $\rightarrow$ Chân **AO2** trên TB6612.
* **Động cơ Bánh Phải:**
  * Dây M1 (Đỏ) $\rightarrow$ Chân **BO1** trên TB6612.
  * Dây M2 (Trắng/Đen) $\rightarrow$ Chân **BO2** trên TB6612.

#### 2. Bốn dây Encoder mỗi bánh (nối vào STM32 Nucleo-L476RG):
| Chân trên Encoder GA12-N20 | Bánh Trái (`TIM2`) | Bánh Phải (`TIM3`) | Chức năng |
| :--- | :--- | :--- | :--- |
| **VCC Encoder** (3.3V) | Lỗ **3V3** (ưu tiên đúng mức logic STM32) | Lỗ **3V3** (ưu tiên đúng mức logic STM32) | Cấp nguồn nuôi cảm biến Hall |
| **GND Encoder** | Lỗ **GND** | Lỗ **GND** | Mass chung với STM32 |
| **Pha A (C1)** | Lỗ **`A0`** (`PA0` / `TIM2_CH1` — CN8) | Lỗ **`D12`** (`PA6` / `TIM3_CH1` — CN5) | Xung kênh 1 (Quadrature x4) |
| **Pha B (C2)** | Lỗ **`A1`** (`PA1` / `TIM2_CH2` — CN8) | Lỗ **`D11`** (`PA7` / `TIM3_CH2` — CN5) | Xung kênh 2 (Quadrature x4) |

> **Kiểm tra chiều Encoder thực tế:**
> Khi bấm **`CAT KHAN CAP (!)`** trên Web (hoặc ở trạng thái dừng motor), dùng tay đẩy xe lăn tiến về phía trước: cả 2 số `Enc L / R` phải **cùng tăng DƯƠNG (+)**. Nếu bánh nào bị âm, đảo dấu `DEFAULT_ENC_LEFT_SIGN` hoặc `DEFAULT_ENC_RIGHT_SIGN` trong `board_config.h`.

---

### Bước 4: Cắm cảm biến góc MPU6050 (4 dây I2C)
Cảm biến góc nghiêng là thành phần bắt buộc để xe đo góc và giữ thăng bằng:

* **VCC** (MPU6050) $\rightarrow$ Lỗ **3V3** hoặc **5V** trên STM32 (thanh nguồn CN6 bên trái)
* **GND** (MPU6050) $\rightarrow$ Lỗ **GND** trên STM32
* **SCL** (MPU6050) $\rightarrow$ Lỗ **D15** (chân `PB8` — lỗ trên cùng thanh CN5 bên phải)
* **SDA** (MPU6050) $\rightarrow$ Lỗ **D14** (chân `PB9` — lỗ thứ 2 từ trên xuống thanh CN5 bên phải)

---

## 2. Cấu hình STM32CubeMX

Cả 2 dự án (**`Project_Self_PID`** và **`Project_Motion`**) đều sử dụng chung cấu hình ngoại vi chuẩn xác như sau:

1. **PWM Động cơ (`TIM1`):**
   * Channels: `Channel 1` (PA8 - PWMA) và `Channel 2` (PA9 - PWMB) chọn `PWM Generation CH1 CH2`, tốc độ chân `GPIO_SPEED_FREQ_HIGH`.
   * `Prescaler (PSC)`: `7` (Xung cấp = 80MHz / (7+1) = 10 MHz).
   * `Counter Period (ARR)`: `999` (Tần số PWM = 10 MHz / (999+1) = **10 kHz**).
   * Kích hoạt `__HAL_TIM_MOE_ENABLE(&htim1)` để xuất xung PWM ra chân chip.
2. **GPIO Output (Hướng động cơ & STBY):**
   * `PC0`: Output (STBY TB6612 - Kéo HIGH).
   * `PB4`, `PB5`: Output (AIN1, AIN2 - Bánh Trái).
   * `PB6`, `PA10`: Output (BIN1, BIN2 - Bánh Phải; `PB7` giữ Reserved LOW).
3. **Giao tiếp I2C cảm biến MPU6050 (`I2C1`):**
   * Pins: `PB8` (I2C1_SCL) và `PB9` (I2C1_SDA) — Open-Drain + Pull-up.
   * Tốc độ `200 kHz` (`Timing = 0x10425F73`).
4. **Định thời vòng điều khiển cân bằng (`TIM6`):**
   * `Prescaler`: `7999` (10 kHz $\rightarrow$ 0.1ms mỗi tick).
   * `Counter Period`: `49` (Chu kỳ = 50 * 0.1ms = **5ms = 200 Hz**).
   * **NVIC:** Tắt `TIM6_DAC_IRQn`, đọc trực tiếp cờ phần cứng `TIM6->SR & TIM_SR_UIF` trong vòng lặp chính (bỏ qua tick lỗi `dt < 1 ms`) để tránh lặp 2 lần liên tiếp hoặc gán sai `dt`.
5. **Đọc Encoder phần cứng (`TIM2` & `TIM3`):**
   * `TIM2` (`PA0`, `PA1` — Bánh Trái) và `TIM3` (`PA6`, `PA7` — Bánh Phải): Chế độ `Encoder Mode TI1 and TI2` (x4, `1400 CPR`, bánh xe `r = 0.022 m`), `IC1/IC2 Filter = 6`, `GPIO_PULLUP`.
   * Chiều đếm tĩnh cố định (`Project_Self_PID`: `LEFT = -1`, `RIGHT = 1`; `Project_Motion`: đã đồng bộ 5 dấu với chiều tiến thực tế `LEFT = 1`, `RIGHT = -1`), tự kích hoạt vòng ngoài khi xác nhận có xung encoder thực (`>= 24` counts) và trong `Project_Motion` không tự tắt encoder khi bánh kẹt.
6. **Debug UART & Điều khiển (`USART2` & `LPUART1`):**
   * `USART2` (`PA2` TX / `PA3` RX, `115200 8N1`): Bật ngắt nhận **`RXNEIE` + ring buffer** trong `USART2_IRQHandler`.
   * `LPUART1` (`PC1` TX / `PB10` RX, `115200 8N1` trong `Project_Motion`): Ngắt `LPUART1_IRQHandler` chạy song song cùng `USART2` để nối ESP32-C6 mà không phụ thuộc cầu nối ST-Link `SB62`/`SB63`.

---

## 3. Chi tiết toàn bộ các File và Thư mục trong Dự án

Trong mỗi dự án STM32CubeIDE, các file được chia làm **4 nhóm rõ rệt**:

### Nhóm 1: File Thuật toán Ứng dụng (Application Code - Do chúng ta viết)
Đây là các file cốt lõi điều khiển xe cân bằng mà bạn sẽ làm việc trực tiếp:

| Tên File | Vị trí | Chức năng chi tiết |
| :--- | :--- | :--- |
| **`mpu6050.h` / `.c`** | `Core/Inc` & `Src` | Đọc dữ liệu gia tốc (Accel) và vận tốc góc (Gyro) qua I2C; tích hợp thuật toán lọc **Kalman Filter 1D** để tính góc nghiêng `pitch_kalman` chính xác, mượt mà và không bị trôi. |
| **`pid.h` / `.c`** | `Core/Inc` & `Src` | Triển khai bộ điều khiển PID số với thuật toán chống bão hòa tích phân (**Anti-windup clamping**), hỗ trợ tính khâu D trực tiếp từ tốc độ góc gyro để triệt tiêu nhiễu giật (**derivative kick**). |
| **`motor.h` / `.c`** | `Core/Inc` & `Src` | Điều khiển xung PWM và chân GPIO chiều quay động cơ qua TB6612FNG; tự động kích hoạt `MOE` cho TIM1 và bù vùng chết mềm tuyến tính (**Deadband**). |
| **`balance_controller.h` / `.c`** | `Core/Inc` & `Src` | Bộ điều khiển cân bằng Cascade (PD/PID góc + vòng vị trí/tốc độ 50 Hz): cắt PWM tạm thời khi nghiêng quá **$35^\circ$** (`FALL_ANGLE_DEG`, tự dựng lại khi $\le 18^\circ$), chốt lỗi phần cứng **`fault_latched`** (`MOTOR_CUT`, `IMU_FAULT`, `I2C_ERR`, `WEB_ESTOP`), rẽ vi sai ưu tiên headroom cân bằng ở `Project_Motion`. |
| **`main.c`** | `Core/Src/` | File thực thi chính: Khởi tạo ngoại vi 1 lần (`SelfBalance_HardwareInit` / `Motion_HardwareInit`), chạy vòng lặp 200 Hz theo cờ `TIM6 UIF`, ngắt motor ngay khi giữ nút `B1`, nhận lệnh qua ring buffer UART (`RXNEIE`) và gửi Telemetry `STAT:...`. |

---

### Nhóm 2: File Cấu hình Hệ thống & Ngắt (Do CubeMX sinh ra trong `Core/`)
Liên kết giữa phần mềm và phần cứng STM32L476:

| Tên File | Vị trí | Chức năng chi tiết |
| :--- | :--- | :--- |
| **`main.h`** | `Core/Inc/` | Định nghĩa các chân IO chung của board mạch (`B1_Pin`, `LD2_Pin`,...) và khai báo callback nhận byte ngắt UART. |
| **`stm32l4xx_it.h` / `.c`** | `Core/Inc` & `Src` | **Interrupt Handlers:** Nơi chứa hàm xử lý ngắt phần cứng `SysTick_Handler`, `USART2_IRQHandler` và `LPUART1_IRQHandler` (`TIM6_DAC_IRQn` tắt vì vòng 200 Hz đọc cờ `UIF` trực tiếp). |
| **`stm32l4xx_hal_conf.h`**| `Core/Inc/` | File cấu hình HAL: Nơi bật/tắt các module ngoại vi của STM32 (`HAL_TIM_MODULE_ENABLED`, `HAL_I2C_MODULE_ENABLED`, `HAL_UART_MODULE_ENABLED`,...). |
| **`stm32l4xx_hal_msp.c`** | `Core/Src/` | **MCU Support Package (MSP):** Cấu hình cấp thấp cho từng ngoại vi: Bật xung Clock bus cho GPIO/Timer/I2C/UART, thiết lập Alternate Function và điện trở Pull-up cho Encoder/I2C/UART. |
| **`system_stm32l4xx.c`**  | `Core/Src/` | Thiết lập tần số xung nhịp hệ thống CPU lên **80 MHz** (từ nguồn HSI qua bộ nhân tần PLL) và khởi tạo bảng vector ngắt. |
| **`sysmem.c` & `syscalls.c`** | `Core/Src/` | Cung cấp các hàm dịch vụ hệ điều hành tối thiểu cho thư viện chuẩn C (như hàm cấp phát bộ nhớ `_sbrk` cho `malloc`, các hàm nhập xuất `_read`, `_write` cho `printf`). |

---

### Nhóm 3: Thư viện chuẩn của STMicroelectronics & ARM (`Drivers/`)
Thư viện nền tảng của nhà sản xuất, **không cần sửa đổi**:

* **`Drivers/STM32L4xx_HAL_Driver/`**: Bộ thư viện Hardware Abstraction Layer của ST (chứa các file C và Header điều khiển ngoại vi như `stm32l4xx_hal_tim.c/.h`, `stm32l4xx_hal_i2c.c/.h`, `stm32l4xx_hal_uart.c/.h`,...).
* **`Drivers/CMSIS/`**: **Cortex Microcontroller Software Interface Standard** do ARM ban hành, định nghĩa các thanh ghi phần cứng lõi Cortex-M4, các hàm hợp ngữ và tập lệnh toán học tối ưu.

---

### Nhóm 4: File Cấu hình Dự án & Nạp Chip (Thư mục gốc)
* **`Project_Self_PID.ioc` / `Project_Motion.ioc`**: File đồ họa của STM32CubeMX lưu trữ toàn bộ cấu hình chân chip, timer (`TIM1/2/3/6`), UART (`USART2`/`LPUART1`) và clock.
* **`STM32L476RGTX_FLASH.ld`**: **Linker Script** hướng dẫn trình biên dịch GCC nạp code vào bộ nhớ Flash (1024 KB) và biến vào bộ nhớ RAM (128 KB).
* **`.project`, `.cproject`, `.mxproject`**: File quản lý dự án của môi trường phát triển Eclipse / STM32CubeIDE.
* **`.settings/`**: Thư mục lưu cấu hình biên dịch riêng của IDE.

---

## 4. Phân biệt & So sánh 2 Dự án (`Project_Self_PID` vs `Project_Motion`)

Trong thư mục hiện tại có sẵn **2 dự án độc lập**, bạn có thể mở cả 2 dự án song song trong STM32CubeIDE:

| Tiêu chí so sánh | `Project_Self_PID` | `Project_Motion` |
| :--- | :--- | :--- |
| **Mục đích chính** | **Chỉ tự cân bằng đứng yên tại chỗ** (tách riêng phần PID thuần) | **Vừa tự cân bằng vừa di chuyển W/A/S/D** |
| **Cơ chế điều khiển** | Cascade PID (`48.00 / 3.00 / 5.60`, `deadband=125` + vòng vị trí 50 Hz) giữ vị trí đứng yên | Cascade PID cứng (`62.00 / 4.00 / 6.60`, `deadband=145`) + PI vận tốc có tích phân chuyển mượt bumpless, 3 mức tốc độ (`0.14 / 0.22 / 0.30 m/s`), rẽ ưu tiên cân bằng |
| **Xử lý UART** | Ngắt `USART2` (`RXNEIE` ring buffer) hỗ trợ lệnh `'x'` (dừng), `'i'` (calib), `'z'` (đặt zero), `'r'` (arm) & đẩy `STAT:` 25 Hz | Ngắt `USART2` + `LPUART1` nhận lệnh từ ESP32-C6 WebSocket & trả `STAT:` kèm `PWM Out`, `Enc L/R`, `Enc State` |
| **Ứng dụng** | Nạp trước để kiểm chứng an toàn (Đợt 1) và chỉnh mốc góc `DEFAULT_ZERO_DEG` | Nạp sau để lái từ xa qua WebSocket trên điện thoại |

### Bảng lệnh UART / WebSocket trong `Project_Motion` (Baud 115200):
* **`1` / `2` / `3`**: Chọn mức tốc độ mục tiêu (**Chậm `0.14 m/s`**, **Vừa `0.22 m/s`**, **Nhanh `0.30 m/s`**).
* **`w` / `s`** (nhả **`v`**): Tiến / Lùi theo mức tốc độ đang chọn (khi giữ nút, Web tự gửi lại chính lệnh `w`/`s` mỗi `150 ms` để mất 1 byte vẫn tự khôi phục); nhả tay trả tốc độ về `0` và phanh mượt về đứng yên.
* **`a` / `d`** (nhả **`c`**): Rẽ trái / Rẽ phải vi sai có co tự động theo PWM headroom cân bằng (tắt rẽ nếu sai lệch góc > $12^\circ$, giữ nguyên vị trí tâm quay).
* **`k`**: Vẫn hỗ trợ gói keep-alive làm mới timeout `500 ms` để tương thích ngược.
* **`x`**: Dừng chạy tiến/lùi/rẽ ngay và tự đứng yên thăng bằng.
* **`r` / `!`**: Mở khóa (`ARM` sau lỗi) hoặc cắt khẩn cấp từ Web (`WEB_ESTOP`).
* **`i` / `z`**: Hiệu chuẩn IMU hoặc lưu mốc góc zero $\le 15^\circ$ (**chỉ cho phép khi xe đang dừng / nằm nghỉ**).

---

## 5. Quy trình mở, biên dịch & nạp code trên STM32CubeIDE

### Cách Import cả 2 dự án vào STM32CubeIDE:
1. Mở **STM32CubeIDE**, vào menu:  
   **`File`** $\rightarrow$ **`Import...`** $\rightarrow$ chọn **`General`** $\rightarrow$ **`Existing Projects into Workspace`** $\rightarrow$ bấm **Next**.
2. Tại ô *Select root directory*, bấm **Browse...** chọn thư mục chứa code:
   * Chọn `Project_Self_PID` $\rightarrow$ bấm **Finish**.
   * Tiếp tục lặp lại và chọn `Project_Motion` $\rightarrow$ bấm **Finish**.
3. Cả 2 dự án sẽ xuất hiện song song trong cửa sổ *Project Explorer* bên trái.

### Các bước nạp code:
1. Nhấp đúp mở dự án tương ứng (`Project_Self_PID` hoặc `Project_Motion`). Nếu mở `.ioc` trên STM32CubeMX nhớ kiểm tra giữ nguyên các hàm ngắt/UART tùy biến trong `USER CODE`.
2. Nhấn **Project $\rightarrow$ Clean...** rồi **`Ctrl + B`** (Build Project) $\rightarrow$ Đảm bảo biên dịch thành công: **0 Errors**.
3. Cắm cáp USB vào mạch Nucleo và nhấn nút **Run** (biểu tượng Play xanh) để nạp code xuống xe.

---

## 6. Quy trình kiểm chứng & hiệu chỉnh (Tune) PID thực tế

> **Lưu ý an toàn:** Luôn thử đứng yên bằng **`Project_Self_PID`** trước rồi mới chuyển sang lái bằng **`Project_Motion`**. Khi nạp code lần đầu, hãy giữ nhẹ 2 bên thân xe trước khi thả tay.

### Bước 1: Xác định điểm cân bằng cơ khí tĩnh (`DEFAULT_ZERO_DEG` / `nominal_balance_angle`)
* Giữ xe đứng thẳng tự nhiên nhất rồi quan sát `Pitch` và `Theta Ref` trên Serial Terminal / Web (`115200` baud).
* Hoặc giữ xe gần thẳng đứng (sai lệch $\le 15^\circ$) rồi gửi lệnh **`z`** khi xe đang dừng để đo tạm mốc cân bằng thực, sau đó cập nhật vào `DEFAULT_ZERO_DEG` trong [`board_config.h`](Core/Inc/board_config.h) (mặc định đang khóa `-3.65f` với `B1_CAPTURE_ZERO_ENABLE = 0` để tránh nhấn `B1` làm lệch mốc đã đo).

### Bước 2: Kiểm tra đúng chiều quay động cơ (Phản hồi âm)
* Cầm xe trên tay, nghiêng nhẹ xe về phía trước: **Bánh xe phải quay về phía trước** để đỡ thân xe.
* Nghiêng xe về sau: **Bánh xe phải quay lùi**.
* *Nếu bánh xe quay ngược chiều (nghiêng tới mà bánh chạy lùi làm xe ngã nhanh hơn):* Đảo 2 dây động cơ ở chân AO1/AO2 (hoặc BO1/BO2) trên TB6612 hoặc chỉnh `DEFAULT_LEFT_SIGN` / `DEFAULT_RIGHT_SIGN` trong `board_config.h`.

### Bước 3: Tinh chỉnh PID trong `board_config.h`
1. Bộ thông số thực nghiệm trên xe (`DEFAULT_KP = 48.00f`, `DEFAULT_KI = 3.00f`, `DEFAULT_KD = 5.60f`, `DEFAULT_DEADBAND = 125`, `DEFAULT_ZERO_DEG = -3.65f`) đã hiệu chỉnh từ thông số nền Simulink để giảm vọt lố và thắng ma sát tĩnh hộp số N20.
2. Nếu xe rung tần số cao: giảm nhẹ `DEFAULT_KP` hoặc tăng `DEFAULT_KD`. Nếu xe phản hồi chậm / dễ ngã: tăng dần `DEFAULT_KP` hoặc kiểm tra `DEFAULT_DEADBAND`.
3. Khi đổi thông số trong [`board_config.h`](Core/Inc/board_config.h), biên dịch và nạp lại cho STM32 (có thể dùng lệnh UART `'z'` khi xe đang `FALL_WAIT`/dừng để test nhanh mốc `zero` tạm trước khi ghi cứng).

---

## 7. Hướng dẫn Mở rộng: Điều khiển qua WiFi bằng ESP32-C6 (WebSocket Controller)

Khi muốn điều khiển xe chạy không dây từ xa bằng điện thoại (cả **iPhone / iOS** và **Android** mà không cần cài app):

```
[ Điện Thoại / Tablet ] ──(WiFi + WebSocket ws://192.168.4.1/ws)──► [ ESP32-C6 Web Server ] ──(UART 4 dây)──► [ STM32 Project_Motion ]
```

### Bước 1: Đấu 4 sợi dây giữa ESP32-C6 SuperMini và STM32 Nucleo
| Chân trên ESP32-C6 | Chân trên STM32 Nucleo | Chức năng |
| :--- | :--- | :--- |
| **5V / VBUS** | Lỗ **5V** (thanh nguồn CN6 bên trái) | Cấp nguồn nuôi mạch ESP32-C6 |
| **GND** | Lỗ **GND** trên STM32 Nucleo | Nối chung đất Mass |
| **GPIO 18** hoặc chân **`TX` (GPIO 16)** | Lỗ **`D6`** (`PB10` RX `LPUART1`, khuyến nghị mặc định) — hoặc `D0` (`PA3`) nếu đã hàn `SB62`/`SB63` | ESP32-C6 truyền lệnh sang STM32 |
| **GPIO 17 (`RX`)** | Lỗ **`A4`** (`PC1` TX `LPUART1`, khuyến nghị mặc định) — hoặc `D1` (`PA2`) nếu đã hàn `SB62`/`SB63` | ESP32-C6 nhận Telemetry `STAT:...` từ STM32 |

### Bước 2: Nạp code cho ESP32-C6 bằng Arduino IDE
1. Mở file `ESP32_WiFi_Controller.ino` trong Arduino IDE (dùng sẵn `<esp_http_server.h>` trong ESP32 Arduino Core 3.x, không cần cài thêm thư viện).
2. Menu **Tools** $\rightarrow$ **Board** $\rightarrow$ chọn **`ESP32C6 Dev Module`**.
3. Chọn cổng COM và bấm nút **Upload** (mũi tên sang phải) để nạp.

### Bước 3: Lái xe trên điện thoại
1. Bật nguồn xe $\rightarrow$ ESP32-C6 tự động phát WiFi tên **`XE_CAN_BANG_WIFI`** (mật khẩu: **`12345678`**).
2. Dùng điện thoại kết nối vào mạng WiFi trên.
3. Mở trình duyệt web (**Safari** hoặc **Chrome**), gõ vào thanh địa chỉ:
   ```
   192.168.4.1
   ```
4. Giao diện WebSocket Dark Mode sẽ hiện lên kèm trạng thái kết nối, góc Pitch, `Theta Ref`, `Zero Góc`, `V Lệnh`, `V Thực`, `PWM Out`, `Enc L / R`, `Enc State`, khung `LOG TELEMETRY` và nút bấm:
   * **Chọn mức tốc độ (`1/2/3`)**: Chậm (`0.14`), Vừa (`0.22`), Nhanh (`0.30` m/s).
   * **Giữ `TIẾN` (`w`) / `LÙI` (`s`) / `TRÁI` (`a`) / `PHẢI` (`d`)**: Web tự gửi lại chính lệnh đang giữ mỗi `150 ms` để chống mất gói, và tự dừng mượt khi nhả tay hoặc mất kết nối.
   * **Bấm `DỪNG` (`x`) / `MỞ KHÓA` (`r`) / `CẮT KHẨN CẤP` (`!`)** và **`Calib IMU` (`i`) / `Set Zero` (`z`)** (chỉ mở khi xe không ở trạng thái đang cân bằng `BALANCE`).

