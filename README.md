# HƯỚNG DẪN CẤU HÌNH & VẬN HÀNH XE TỰ CÂN BẰNG (STM32L476RG)

Tài liệu hướng dẫn toàn diện từ sơ đồ đấu nối phần cứng, cấu hình ngoại vi STM32CubeMX, giải thích chi tiết **tất cả các file trong dự án** và hướng dẫn sử dụng 2 dự án: **`Project_Self_PID`** (Tự cân bằng đứng yên "đóng đinh tại chỗ") và **`Project_Motion`** (Vừa cân bằng vừa di chuyển qua ESP32-C6 WebSocket).

---

## Cấu trúc nhánh (Git Branches)

Repo sử dụng mô hình **1 nhánh khung nền (`main`) + 2 nhánh chức năng độc lập** ngay tại thư mục gốc (không bọc lồng thư mục con):

| Nhánh | Nội dung (tại thư mục gốc `/`) | Cách chuyển nhánh |
| :--- | :--- | :--- |
| **`main`** | **Bộ khung phần cứng & driver nền STM32L476RG**: Clock 80 MHz, `TIM1` PWM 10 kHz (soft deadband, không `EGR_UG` khi đảo chiều), `TIM2`/`TIM3` Encoder, `TIM6` 200 Hz, `I2C1` **ICM-20602** (DWT 2.5 ms + Kalman chống lệch gia tốc), `motor.c`, `pid.c`, `USART2` | `git checkout main` |
| **`Project_self_PID`** | Firmware hoàn chỉnh **tự cân bằng đứng yên tại chỗ** (PD góc thuần + PID vị trí $v_{\text{com}}$ + khóa hướng Yaw `enc_L - enc_R` & `imu.gz` + `TIM6`/`IWDG` Watchdog + `STBY=LOW` khi ngã + `LPUART1` + `Project_Self_PID.ioc`) | `git checkout Project_self_PID` |
| **`Project_motion`** | Firmware hoàn chỉnh **cân bằng + điều khiển di chuyển `W/A/S/D`** (PD góc + PI vận tốc $v_{\text{com}}$ + khóa hướng `imu.gz` + chống lệch Kalman + `LPUART1` + `Project_Motion.ioc` + `ESP32_WiFi_Controller/`) | `git checkout Project_motion` |

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
> `D10` (BIN1) $\rightarrow$ (trống D9) $\rightarrow$ `D8` (PWMB) $\rightarrow$ `D7` (PWMA) $\rightarrow$ `D6` (UART RX từ ESP32-C6) $\rightarrow$ `D5` (AIN1) $\rightarrow$ `D4` (AIN2) $\rightarrow$ (trống D3) $\rightarrow$ `D2` (BIN2).

---

### Bước 3: Nối 2 động cơ GA12-N20 kèm Encoder (6 dây mỗi bánh)
Mỗi động cơ GA12-N20 có cụm 6 sợi dây gồm **2 dây cấp nguồn motor ở ngoài cùng** và **4 dây Encoder ở giữa**:

#### 1. Hai dây nguồn động cơ (nối vào TB6612FNG):
* **Động cơ Bánh Trái:** M1 $\rightarrow$ **AO1**, M2 $\rightarrow$ **AO2** trên TB6612.
* **Động cơ Bánh Phải:** M1 $\rightarrow$ **BO1**, M2 $\rightarrow$ **BO2** trên TB6612.

#### 2. Bốn dây Encoder mỗi bánh (nối vào STM32 Nucleo-L476RG):
| Chân trên Encoder GA12-N20 | Bánh Trái (`TIM2`) | Bánh Phải (`TIM3`) | Chức năng |
| :--- | :--- | :--- | :--- |
| **VCC Encoder** (3.3V) | Lỗ **3V3** | Lỗ **3V3** | Cấp nguồn nuôi cảm biến Hall |
| **GND Encoder** | Lỗ **GND** | Lỗ **GND** | Mass chung với STM32 |
| **Pha A (C1)** | Lỗ **`A0`** (`PA0` / `TIM2_CH1` — CN8) | Lỗ **`D12`** (`PA6` / `TIM3_CH1` — CN5) | Xung kênh 1 (Quadrature x4) |
| **Pha B (C2)** | Lỗ **`A1`** (`PA1` / `TIM2_CH2` — CN8) | Lỗ **`D11`** (`PA7` / `TIM3_CH2` — CN5) | Xung kênh 2 (Quadrature x4) |

> **Kiểm tra chiều Encoder thực tế:**  
> Khi ở trạng thái dừng motor (`DISARMED` / `!`), dùng tay đẩy xe lăn tiến về phía trước: cả 2 số `Enc L / R` phải **cùng tăng DƯƠNG (+)** (`DEFAULT_ENC_LEFT_SIGN = 1`, `DEFAULT_ENC_RIGHT_SIGN = -1`).

---

### Bước 4: Cắm cảm biến góc ICM-20602 (I2C)
Cảm biến ICM-20602 giao tiếp I2C1 chuẩn (`WHO_AM_I = 0x12`, địa chỉ `0x68`):

* **VCC** (ICM-20602) $\rightarrow$ Lỗ **3V3** (hoặc **5V** nếu module có IC hạ áp tích hợp)
* **GND** (ICM-20602) $\rightarrow$ Lỗ **GND** trên STM32
* **CS** (ICM-20602) $\rightarrow$ Nối **3V3** (khóa cứng chế độ I2C)
* **SA0 / AD0** (ICM-20602) $\rightarrow$ Nối **GND** (cố định địa chỉ I2C `0x68`)
* **SCL** (ICM-20602) $\rightarrow$ Lỗ **D15** (`PB8` — I2C1_SCL)
* **SDA** (ICM-20602) $\rightarrow$ Lỗ **D14** (`PB9` — I2C1_SDA)

---

## 2. Cấu hình STM32CubeMX

Cả 2 dự án (**`Project_Self_PID`** và **`Project_Motion`**) sử dụng chung cấu hình ngoại vi:

1. **PWM Động cơ (`TIM1`):** `PA8` (`CH1`), `PA9` (`CH2`), `PSC = 7`, `ARR = 999` (**10 kHz**), bật `__HAL_TIM_MOE_ENABLE(&htim1)`. Không ghi `TIM_EGR_UG` khi đảo chiều để tránh giật xung quanh điểm 0.
2. **GPIO Output (Hướng động cơ & STBY):** `PC0` (`STBY`), `PB4`/`PB5` (`AIN1`/`AIN2`), `PB6`/`PA10` (`BIN1`/`BIN2`), `PB7` giữ Reserved LOW.
3. **Giao tiếp I2C cảm biến ICM-20602 (`I2C1`):** `PB8` (`SCL`) và `PB9` (`SDA`), `200 kHz` (`Timing = 0x10425F73`), đọc Runtime 15 byte (`0x3A..0x48`) có giới hạn cứng **2.5 ms bằng DWT** để loại bỏ bẫy chờ 25 ms của HAL.
4. **Định thời & Watchdog (`TIM6` & `IWDG`):** `TIM6` `PSC = 7999`, `ARR = 49` (**200 Hz = 5 ms**), bật ngắt cứng `TIM6_DAC_IRQn` priority `0` kèm ISR Watchdog `30 ms` (`LOOP_WDT`) và `IWDG` `500 ms`.
5. **Đọc Encoder phần cứng (`TIM2` & `TIM3`):** `Encoder Mode TI1 and TI2` (x4, `1400 CPR`, bánh xe `r = 0.022 m`), `IC1/IC2 Filter = 6`, đồng bộ 5 dấu theo chiều tiến thực tế (`ENC_L = 1`, `ENC_R = -1`, `CONTROL = -1`, `LEFT = -1`, `RIGHT = -1`).
6. **Debug UART & Điều khiển (`USART2` & `LPUART1`):** Chạy song song **`USART2`** (`PA2`/`PA3`, `115200 8N1`) và **`LPUART1`** (`PC1` TX `A4` / `PB10` RX `D6`, `115200 8N1`) qua ngắt `RXNEIE` ring buffer và truyền Telemetry 25 Hz non-blocking 17 trường.

---

## 3. Chi tiết toàn bộ các File ứng dụng (`Core/`)

| Tên File | Vị trí | Chức năng chi tiết |
| :--- | :--- | :--- |
| **`icm20602.h` / `.c`** | `Core/Inc` & `Src` | Driver **ICM-20602** (`WHO_AM_I = 0x12`, `SIGNAL_PATH_RESET`, Gyro DLPF `92 Hz`, Accel DLPF `44.8 Hz`), đọc I2C bounded DWT 2.5 ms, kiểm tra đứng yên khi calib (`300` mẫu, `IMU_MOVING`), lọc `gyro_rate_yaw` và **Kalman 1D bù gia tốc ngang + khóa bias**. |
| **`pid.h` / `.c`** | `Core/Inc` & `Src` | PID số với Anti-windup clamping, dùng vận tốc góc Gyro cho khâu D và tự động ngắt tích phân khi `Ki == 0` (vòng trong PD thuần). |
| **`motor.h` / `.c`** | `Core/Inc` & `Src` | Điều khiển TB6612FNG, bù vùng chết mềm tuyến tính (`DEFAULT_DEADBAND = 130`, `MOTOR_DEADBAND_RAMP = 35`), cập nhật trực tiếp CCR không kích `TIM_EGR_UG` khi qua 0, đọc Encoder 16-bit chống tràn. |
| **`balance_controller.h` / `.c`** | `Core/Inc` & `Src` | Cascade PD góc (200 Hz) + vòng ngoài 50 Hz bù khối tâm $v_{\text{com}}$ + khóa hướng Yaw bằng `imu.gz` + giám sát mất xung encoder (`ENC_LOSS_L/R`, `ENC_NO_SIG`, `MOTOR_STALL`). |
| **`main.c`** | `Core/Src/` | Vòng điều khiển cứng 200 Hz qua ngắt `TIM6`, `IWDG` 500 ms, nút nhấn `B1` cắt khẩn cấp / calib, Telemetry 25 Hz qua `USART2` & `LPUART1`. |

---

## 4. Phân biệt & So sánh 2 Dự án (`Project_Self_PID` vs `Project_Motion`)

| Tiêu chí so sánh | `Project_Self_PID` | `Project_Motion` |
| :--- | :--- | :--- |
| **Mục đích chính** | **Thuần tự cân bằng đứng yên tại chỗ** | **Vừa tự cân bằng vừa di chuyển `W/A/S/D`** |
| **Vòng trong (200 Hz)** | PD thuần (`Kp = 62.00`, `Ki = 0.00`, `Kd = 6.60`, `Deadband = 130`, ramp `35`) | PD thuần (`Kp = 62.00`, `Ki = 0.00`, `Kd = 6.60`, `Deadband = 130`, ramp `35`) |
| **Vòng ngoài (50 Hz)** | PID vị trí khối tâm ($x_{\text{com}}, v_{\text{com}}$) với `Kp_pos = 2.40`, `Ki_pos = 0.25`, `Kd_pos = 3.40` (giữ `integral` khi rebase `> 0.60 m`) | Đứng yên: PID $x_{\text{com}}$ (reset sạch `integral` sau phanh); Di chuyển: PI vận tốc $v_{\text{com}}$ (`Kp_vel = 5.5`, `Ki_vel = 6.5`), ăn lệnh `W/S` ngay từ `0` xung |
| **Khóa hướng Yaw** | PD khóa hướng tại chỗ (`enc_L - enc_R` + `imu.gz`), cắt khi nghiêng $> 12^\circ$ | Khóa hướng đi thẳng & bám tốc độ góc rẽ `±95 dps` bằng `imu.gz`, ưu tiên headroom cân bằng |
| **An toàn & Khởi động** | Nằm nghiêng $\rightarrow$ `FALL_WAIT` (`STBY = LOW`), dựng thẳng $\le 15^\circ$ tự đứng; lỗi phần cứng chốt `fault_latched` | Khởi động luôn giữ `DISARMED` (`STBY = LOW`), chỉ chạy khi có lệnh `'r'` hợp lệ; ngã $> 35^\circ$ chốt `FALL_LATCH` |
| **Telemetry & Điều khiển** | `USART2` + `LPUART1` xuất `STAT:` 17 trường, nhận `!`, `x`, `i`, `z`, `r` | `USART2` + `LPUART1` xuất `STAT:` 17 trường, nhận đầy đủ `1/2/3`, `w/s/v`, `a/d/c`, `x`, `r`, `!`, `i`, `z` qua ESP32-C6 WebSocket |

---

## 5. Quy trình mở, biên dịch & nạp code trên STM32CubeIDE

1. Chọn nhánh cần nạp: `git checkout Project_self_PID` hoặc `git checkout Project_motion`.
2. Trong STM32CubeIDE: **`File` $\rightarrow$ `Import...` $\rightarrow$ `General` $\rightarrow$ `Existing Projects into Workspace`**.
3. Sau khi cập nhật driver sang `icm20602.c`, bấm chuột phải dự án chọn **`Refresh (F5)`**, chạy **`Project` $\rightarrow$ `Clean...`** rồi **`Ctrl + B` (Build Project)** để đảm bảo **0 Errors** trước khi bấm **Run/Debug**.

---

## 6. Quy trình kiểm chứng nguội & hiệu chỉnh thực tế (ICM-20602)

> **Lưu ý an toàn:** Với `IMU_CALIB_SAMPLES = 300U`, giữ xe nằm yên hoàn toàn khoảng **0.7–1 giây** lúc mới cấp nguồn.

1. **Kiểm tra nguội khi đang `DISARMED` (chưa bấm `ARM`):**
   * Dựng xe thẳng đứng $\rightarrow$ `Goc Pitch` phải gần $0^\circ$ (trong $\pm 10^\circ$).
   * Nghiêng xe về phía **TIẾN** $\rightarrow$ `Goc Pitch` phải **tăng dương** (khớp `IMU_TILT_AXIS = 1`, `DEFAULT_CONTROL_SIGN = -1`).
   * Mặt chip ICM-20602 hướng lên $\rightarrow$ giữ `DEFAULT_YAW_SIGN = -1` (nếu gắn úp mặt chip xuống gầm thì đảo thành `+1`).
2. **Mốc `DEFAULT_ZERO_DEG`:** Đang đồng bộ **`+1.50f`** (`B1_CAPTURE_ZERO_ENABLE = 0`). Có thể test nhanh bằng nút `Set Zero (Z)` trên Web UI khi xe đang dừng rồi cập nhật vào [`board_config.h`](Core/Inc/board_config.h).

---

## 7. Điều khiển qua WiFi bằng ESP32-C6 (WebSocket Controller)

1. Nối 4 dây giữa ESP32-C6 và STM32 Nucleo: `5V -> 5V`, `GND -> GND`, `GPIO 18 (TX) -> D6 (PB10)`, `GPIO 17 (RX) -> A4 (PC1)`.
2. Nạp [`ESP32_WiFi_Controller.ino`](ESP32_WiFi_Controller/ESP32_WiFi_Controller.ino) bằng Arduino IDE (`ESP32C6 Dev Module`).
3. Kết nối WiFi **`XE_CAN_BANG_WIFI`** (mật khẩu **`12345678`**) và mở trình duyệt tới **`http://192.168.4.1`** để xem Telemetry 25 Hz (góc Pitch, `Theta Ref`, `V Lệnh`, `V Thực`, `PWM Out`, `Enc L/R`, `Loop us`, `Miss`, `IMU age`, `Sat`) và điều khiển `W/A/S/D`, `1/2/3`, `ARM (r)`, `ESTOP (!)`.
