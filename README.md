# HƯỚNG DẪN CẤU HÌNH & VẬN HÀNH XE TỰ CÂN BẰNG (STM32L476RG)

Tài liệu hướng dẫn toàn diện từ sơ đồ đấu nối phần cứng, cấu hình ngoại vi STM32CubeMX, giải thích chi tiết **tất cả các file trong dự án** và hướng dẫn sử dụng **3 dự án STM32**: **`Project_Self_PID`** (Tự cân bằng đứng yên "đóng đinh tại chỗ"), **`Project_Motion`** (Vừa cân bằng vừa di chuyển qua ESP32-C6 WebSocket) và **`Project_self_tuning`** kèm **`Motor_PID_Panel`** (Tinh chỉnh tham số PID trực tiếp qua UART + giao diện Windows GUI).

---

## Cấu trúc nhánh (Git Branches)

Repo sử dụng mô hình **1 nhánh khung nền (`main`) + 3 nhánh chức năng độc lập** ngay tại thư mục gốc (không bọc lồng thư mục con):

| Nhánh | Nội dung (tại thư mục gốc `/`) | Cách chuyển nhánh |
| :--- | :--- | :--- |
| **`main`** | **Bộ khung phần cứng & driver nền STM32L476RG**: Clock 80 MHz, `TIM1` PWM 10 kHz (soft deadband `110`/`48`, không `EGR_UG` khi đảo chiều), `TIM2`/`TIM3` Encoder x4, `TIM6` 200 Hz, `I2C1` **ICM-20602 @ 100 kHz + `DNF=10`** (DWT `2.2 ms` + retry `1.8 ms` + fixed gyro bias 12000 mẫu + Kalman chống lệch gia tốc), `motor.c`, `pid.c`, `USART2` | `git checkout main` |
| **`Project_self_PID`** | Firmware hoàn chỉnh **tự cân bằng đứng yên tại chỗ** (VM `6.0V` CAD `Khung_Frame.step`: PD góc `92/0/5.5` + PID vị trí $v_{\text{com}}$ `9.0/1.8/12.5` + lọc `a_wheel_filt` + khóa hướng Yaw `0.30/1.20` + `TIM6`/`IWDG` Watchdog + `STBY=LOW` khi ngã + `LPUART1` + `Project_Self_PID.ioc`) | `git checkout Project_self_PID` |
| **`Project_motion`** | Firmware hoàn chỉnh **cân bằng + điều khiển di chuyển `W/A/S/D`** (VM `6.0V` CAD: PD góc `92/0/5.5` + PID vị trí `9.0/1.8/12.5` + PI vận tốc $v_{\text{com}}$ `9.0/9.0` + khóa hướng `imu.gz` + `LPUART1` + `Project_Motion.ioc` + `ESP32_WiFi_Controller/`) | `git checkout Project_motion` |
| **`Project_self_tuning`** | Firmware **tinh chỉnh PID trực tiếp qua UART (`USART2` / ST-LINK VCP)** (`app.c`, `balance_controller.c`, `Project_self_tuning.ioc`, bộ `tests/` unit test trên host, `build_check.ps1`) + ứng dụng Windows **`Motor_PID_Panel/`** (`v4.2` chỉnh live `PID`/`PPID`/`YPID`/`DEAD`/`RAMP`/`ZERO`, preset `6.0V`, đồ thị & log) | `git checkout Project_self_tuning` |

---

## 1. Sơ đồ đấu nối dây phần cứng (Wiring Diagram)

> **Lưu ý an toàn:** Hãy tháo pin 18650 ra khỏi đế pin trong lúc cắm dây để tránh chạm chập gây hỏng linh kiện.

### Bước 1: Nguồn Pin cho TB6612FNG & STM32 (VM ổn áp 6.0V cho động cơ)
| Chân / Jumper | Nối đến đâu? | Vị trí trên STM32 Nucleo-L476RG |
| :--- | :--- | :--- |
| **VM** (TB6612) | Nguồn ổn áp **`6.0V`** (từ Pin 2S 18650 qua mạch hạ áp) | Cấp riêng cho công suất động cơ trên TB6612 |
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
| **VCC Encoder** (3.3V / 5V) | Lỗ **3V3** (hoặc **5V**, chân `FT`) | Lỗ **3V3** (hoặc **5V**, chân `FT`) | Cấp nguồn nuôi cảm biến Hall |
| **GND Encoder** | Lỗ **GND** | Lỗ **GND** | Mass chung với STM32 |
| **Pha A (C1)** | Lỗ **`A0`** (`PA0` / `TIM2_CH1` — CN8) | Lỗ **`D12`** (`PA6` / `TIM3_CH1` — CN5) | Xung kênh 1 (Quadrature x4) |
| **Pha B (C2)** | Lỗ **`A1`** (`PA1` / `TIM2_CH2` — CN8) | Lỗ **`D11`** (`PA7` / `TIM3_CH2` — CN5) | Xung kênh 2 (Quadrature x4) |

> **Kiểm tra chiều Encoder thực tế:**  
> Khi ở trạng thái dừng motor (`DISARMED` / `!` / `X`), dùng tay đẩy xe lăn tiến về phía trước: cả 2 số `Enc L / R` phải **cùng tăng DƯƠNG (+)** (`DEFAULT_ENC_LEFT_SIGN = 1`, `DEFAULT_ENC_RIGHT_SIGN = -1`).

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

## 2. Cấu hình STM32CubeMX & Đồng bộ phần cứng

Cả 3 dự án (**`Project_Self_PID`**, **`Project_Motion`**, **`Project_self_tuning`**) được đồng bộ chung bộ driver nền và thông số khung cơ khí `Khung_Frame.step` (`VM = 6.0V`, `l_com = 0.033 m`, vệt bánh `150 mm`):

1. **PWM Động cơ (`TIM1`):** `PA8` (`CH1`), `PA9` (`CH2`), `PSC = 7`, `ARR = 999` (**10 kHz**), bật `__HAL_TIM_MOE_ENABLE(&htim1)`. Soft deadband tuyến tính `DEFAULT_DEADBAND = 110`, `MOTOR_DEADBAND_RAMP = 48`, cắt vùng nhiễu nhỏ `|val| <= 2` và không ghi `TIM_EGR_UG` khi đảo chiều.
2. **GPIO Output (Hướng động cơ & STBY):** `PC0` (`STBY`), `PB4`/`PB5` (`AIN1`/`AIN2`), `PB6`/`PA10` (`BIN1`/`BIN2`), `PB7` giữ Reserved LOW.
3. **Giao tiếp I2C cảm biến ICM-20602 (`I2C1`):** `PB8` (`SCL`) và `PB9` (`SDA`), **`100 kHz`** (`IMU_I2C_TIMING = 0x30955763U`) + bộ lọc số chống nhiễu EMI **`DNF = 10`** (`125 ns`). Đọc Runtime 15 byte (`0x3A..0x48`) có giới hạn cứng **2.2 ms bằng DWT**, tự phục hồi bus (tối đa 16 xung SCL) và chỉ retry thêm 1 lần khi tổng thời gian `< 1.8 ms` (`IMU_RETRY_BUDGET_US`), chịu tối đa `6` chu kỳ mất mẫu (`IMU_MAX_AGE_MS = 35 ms`).
4. **Cấu hình DLPF & Fixed Gyro Bias (`ICM-20602`):** Gyro DLPF `92 Hz / 3.9 ms` (`0x1A = 2`), Accel DLPF `44.8 Hz` (`0x1D = 3`), `GYRO_LPF_ALPHA = 0.55`, khóa 3 offset gyro LSB đo tĩnh `12000` mẫu (`log 20261127_141016` @ `29.39°C`: `X = -8.784917`, `Y = 21.700750`, `Z = -14.417417`), kèm lọc Kalman 1D bù gia tốc ngang `a_wheel_filt`.
5. **Định thời & Watchdog (`TIM6` & `IWDG`):** `TIM6` `PSC = 7999`, `ARR = 49` (**200 Hz = 5 ms**), ngắt cứng `TIM6_DAC_IRQn` kèm kiểm tra deadline (`CONTROL_MAX_MISSED_TICKS = 3`). (`Project_Self_PID` & `Project_Motion` bật thêm ISR Watchdog `30 ms` và `IWDG` `500 ms`).
6. **Đọc Encoder phần cứng (`TIM2` & `TIM3`):** `Encoder Mode TI1 and TI2` (x4, `1400 CPR`, bánh xe `r = 0.022 m`), `IC1/IC2 Filter = 6`, đồng bộ 5 dấu theo chiều tiến thực tế (`ENC_L = 1`, `ENC_R = -1`, `CONTROL = -1`, `LEFT = -1`, `RIGHT = -1`).
7. **UART Debug / Điều khiển / Tuning:** `USART2` (`PA2`/`PA3`, `115200 8N1`) cho ST-LINK VCP (`Motor_PID_Panel` trên `Project_self_tuning`); `Project_Self_PID` & `Project_Motion` chạy song song thêm `LPUART1` (`PC1` TX `A4` / `PB10` RX `D6`) kết nối ESP32-C6.

---

## 3. Chi tiết toàn bộ các File ứng dụng (`Core/`)

| Tên File | Vị trí | Chức năng chi tiết |
| :--- | :--- | :--- |
| **`icm20602.h` / `.c`** | `Core/Inc` & `Src` | Driver **ICM-20602** (`WHO_AM_I = 0x12`, `SIGNAL_PATH_RESET`, Gyro DLPF `92 Hz`, Accel DLPF `44.8 Hz`), `I2C1 100 kHz + DNF=10`, đọc DWT bounded `2.2 ms` + bus recovery 16 xung + retry `1.8 ms`, fixed gyro bias 12000 mẫu, kiểm tra đứng yên khi calib (`300` mẫu, `IMU_MOVING`), lọc `gyro_rate_yaw` và **Kalman 1D bù gia tốc ngang + khóa bias**. |
| **`pid.h` / `.c`** | `Core/Inc` & `Src` | PID số với Anti-windup clamping, dùng vận tốc góc Gyro cho khâu D và tự động ngắt tích phân khi `Ki == 0` (vòng trong PD thuần). |
| **`motor.h` / `.c`** | `Core/Inc` & `Src` | Điều khiển TB6612FNG, bù vùng chết mềm tuyến tính (`DEFAULT_DEADBAND = 110`, `MOTOR_DEADBAND_RAMP = 48`, cắt nhiễu `|val| <= 2`), cập nhật trực tiếp CCR không kích `TIM_EGR_UG` khi qua 0, đọc Encoder 16-bit chống tràn. |
| **`balance_controller.h` / `.c`** | `Core/Inc` & `Src` | Cascade PD góc (200 Hz) + vòng ngoài 50 Hz bù khối tâm $v_{\text{com}}$ (`l_com = 0.033 m`) + lọc thông thấp `a_wheel_filt` cho Kalman + khóa hướng Yaw bằng `imu.gz`. |
| **`app.h` / `.c`** *(nhánh `Project_self_tuning`)* | `Core/Inc` & `Src` | Giao thức UART `PID_PANEL PROTO=3` cho tinh chỉnh live (`PID`, `PPID`, `YPID`, `ZERO`/`SETZERO`, `DEAD`, `RAMP`, `LIMIT`, `M`, `ARM`, `X`, `C`, `STATUS`, `DIAG`, `LOG`), vòng lặp 200 Hz + watchdog + giám sát deadline. |
| **`main.c`** | `Core/Src/` | Khởi tạo ngoại vi (`I2C1` theo `IMU_I2C_TIMING` & `IMU_I2C_DNF`), vòng điều khiển cứng 200 Hz qua ngắt `TIM6`, nút nhấn `B1`, Telemetry / giao thức điều khiển. |

---

## 4. So sánh 3 Dự án (`Project_Self_PID` vs `Project_Motion` vs `Project_self_tuning`)

| Tiêu chí so sánh | `Project_Self_PID` | `Project_Motion` | `Project_self_tuning` |
| :--- | :--- | :--- | :--- |
| **Mục đích chính** | **Thuần tự cân bằng đứng yên tại chỗ** | **Vừa tự cân bằng vừa di chuyển `W/A/S/D`** | **Tinh chỉnh tham số live qua PC (`Motor_PID_Panel`)** |
| **Vòng trong (200 Hz)** | PD thuần (`Kp=92`, `Ki=0`, `Kd=5.5`, `DB=110`, `Ramp=48`) | PD thuần (`Kp=92`, `Ki=0`, `Kd=5.5`, `DB=110`, `Ramp=48`) | PD chỉnh live (`92/0/5.5`, `DB=110`, `Ramp=48` mặc định) |
| **Vòng ngoài (50 Hz)** | PID vị trí $x_{\text{com}}, v_{\text{com}}$ (`9.0 / 1.8 / 12.5`, giữ `integral` khi rebase `> 0.60 m`) | Đứng yên: PID $x_{\text{com}}$ (`9.0 / 1.8 / 12.5`); Di chuyển: PI vận tốc (`Kp_vel=9.0`, `Ki_vel=9.0`) | PID vị trí $x_{\text{com}}, v_{\text{com}}$ (`9.0 / 1.8 / 12.5`), bật/tắt & chỉnh live bằng `ENC`/`PPID` |
| **Khóa hướng Yaw** | PD tại chỗ (`0.30 / 1.20`), cắt khi nghiêng $> 12^\circ$ | Khóa hướng đi thẳng & rẽ `±95 dps` (`Kp_yaw=1.20`, `Ki_hdg=8.00`) | PD tại chỗ (`0.30 / 1.20`), chỉnh live bằng `YPID`/`YSIGN` |
| **ICM-20602 & I2C1** | `100 kHz + DNF=10`, DLPF `92 / 44.8 Hz`, fixed gyro bias, DWT `2.2 ms` / retry `1.8 ms`, `miss<=6`, `age<=35 ms` | Đồng bộ 100% | Đồng bộ 100% |
| **An toàn & Khởi động** | Nằm nghiêng $\rightarrow$ `FALL_WAIT` (`STBY=LOW`), dựng $\le 15^\circ$ tự đứng; lỗi phần cứng chốt `fault_latched` | Luôn giữ `DISARMED` (`STBY=LOW`), chỉ chạy khi nhận `'r'`; ngã $> 35^\circ$ chốt `FALL_LATCH` | Khởi động giữ motor TẮT (`STOPPED`), chỉ chạy khi gửi `ARM` hoặc `M`; ngắt cứng tức thì bằng `X` |
| **Giao tiếp / Công cụ đi kèm** | `USART2` + `LPUART1` Telemetry 25 Hz (`STAT:` 17 trường) | `USART2` + `LPUART1` + **`ESP32_WiFi_Controller/`** WebSocket UI | `USART2` (`PID_PANEL PROTO=3`) + **`Motor_PID_Panel/`** Windows GUI + **`tests/`** host unit tests |

---

## 5. Quy trình mở, biên dịch & nạp code trên STM32CubeIDE

1. Chọn nhánh cần nạp: `git checkout Project_self_PID`, `git checkout Project_motion` hoặc `git checkout Project_self_tuning`.
2. Trong STM32CubeIDE: **`File` $\rightarrow$ `Import...` $\rightarrow$ `General` $\rightarrow$ `Existing Projects into Workspace`**.
3. Bấm chuột phải dự án chọn **`Refresh (F5)`**, chạy **`Project` $\rightarrow$ `Clean...`** rồi **`Ctrl + B` (Build Project)** để đảm bảo **0 Errors** trước khi bấm **Run/Debug**.

---

## 6. Quy trình kiểm chứng nguội & hiệu chỉnh thực tế (ICM-20602)

> **Lưu ý an toàn:** Với `IMU_CALIB_SAMPLES = 300U` và `IMU_USE_FIXED_GYRO_BIAS = 1`, giữ xe nằm yên hoàn toàn khoảng **0.7–1 giây** lúc mới cấp nguồn để kiểm tra đứng yên (`IMU_MOVING`) và khởi tạo góc ban đầu từ gia tốc kế.

1. **Kiểm tra nguội khi đang `DISARMED` / `STOPPED`:**
   * Dựng xe thẳng đứng $\rightarrow$ `Goc Pitch` phải gần $0^\circ$ (trong $\pm 10^\circ$).
   * Nghiêng xe về phía **TIẾN** $\rightarrow$ `Goc Pitch` phải **tăng dương** (khớp `IMU_TILT_AXIS = 1`, `DEFAULT_CONTROL_SIGN = -1`).
   * Mặt chip ICM-20602 hướng lên $\rightarrow$ giữ `DEFAULT_YAW_SIGN = -1` (nếu gắn úp mặt chip xuống gầm thì đảo thành `+1`).
2. **Mốc `DEFAULT_ZERO_DEG`:** Đang đồng bộ **`+1.50f`** (`B1_CAPTURE_ZERO_ENABLE = 0`). Có thể chỉnh nhanh bằng `SETZERO` / `ZERO` trên `Motor_PID_Panel` hoặc nút `Set Zero (Z)` trên Web UI rồi cập nhật vào [`board_config.h`](Core/Inc/board_config.h).

---

## 7. Điều khiển qua WiFi bằng ESP32-C6 (nhánh `Project_motion`)

1. Nối 4 dây giữa ESP32-C6 và STM32 Nucleo: `5V -> 5V`, `GND -> GND`, `GPIO 18 (TX) -> D6 (PB10)`, `GPIO 17 (RX) -> A4 (PC1)`.
2. Chuyển sang nhánh `Project_motion` và nạp `ESP32_WiFi_Controller/ESP32_WiFi_Controller.ino` bằng Arduino IDE (`ESP32C6 Dev Module`).
3. Kết nối WiFi **`XE_CAN_BANG_WIFI`** (mật khẩu **`12345678`**) và mở trình duyệt tới **`http://192.168.4.1`** để xem Telemetry 25 Hz và điều khiển `W/A/S/D`, `1/2/3`, `ARM (r)`, `ESTOP (!)` (xem thêm `HUONG_DAN_ESP32_WIFI.md` trên nhánh `Project_motion`).

---

## 8. Tinh chỉnh PID trực tiếp & Kiểm thử tự động (nhánh `Project_self_tuning`)

Chuyển sang nhánh **`Project_self_tuning`** để dùng đồng thời firmware tuning STM32 và bảng điều khiển Windows **`Motor_PID_Panel`**:

1. **Chạy bảng điều khiển Windows (`Motor_PID_Panel/`):**
   * Đóng PuTTY / Serial Monitor trước khi mở `Motor_PID_Panel/Motor_PID_Panel.exe`.
   * Chọn cổng COM của ST-LINK VCP (`115200 8N1`), bấm **Kết nối**, dùng các nút **Preset Êm / Chuẩn / Cứng (6.0V)** hoặc chỉnh trực tiếp `PID`, `PPID`, `YPID`, `ZERO`, `DEAD`, `RAMP`, `LIMIT`.
   * Biên dịch lại Panel (nếu sửa `Program.cs`):
     ```powershell
     .\Motor_PID_Panel\build.ps1
     .\Motor_PID_Panel\Motor_PID_Panel.exe --self-test .\Motor_PID_Panel\SELF_TEST.txt
     .\Motor_PID_Panel\Motor_PID_Panel.exe --ui-test .\Motor_PID_Panel\UI_TEST.txt
     ```
2. **Chạy kiểm thử firmware tự động:**
   * Host unit tests (giả lập HAL): `.\tests\run_tests.ps1`
   * Kiểm tra biên dịch ARM (`-Wall -Wextra -Werror`): `.\build_check.ps1`
