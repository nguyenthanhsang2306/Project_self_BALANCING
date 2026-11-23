# HƯỚNG DẪN DÙNG ESP32-C6 / ESP32-S3 ĐIỀU KHIỂN XE CÂN BẰNG (`Project_Motion`)

Tài liệu hướng dẫn kết nối **ESP32-C6** (hoặc **ESP32-S3**) làm Web Server phát WiFi để điều khiển xe tự cân bằng **STM32L476RG (`Project_Motion`)** qua điện thoại.

---

## 1. Sơ đồ đấu nối dây giữa ESP32-C6 / ESP32-S3 và STM32 Nucleo-L476RG

Chỉ cần **4 sợi dây Dupont** nối giữa 2 bo mạch đúng như bảng của bạn:

| Chân trên ESP32-C6 / ESP32-S3 | Chân trên STM32 Nucleo-L476RG | Chức năng |
| :--- | :--- | :--- |
| **5V / VIN / VBUS** | Lỗ **5V** (thanh nguồn CN6 bên trái Nucleo) | Cấp nguồn nuôi bo ESP32-C6 |
| **GND** | Lỗ **GND** (trên board STM32 Nucleo) | Nối chung đất Mass |
| **GPIO 18** hoặc chân **`TX` (GPIO 16)** | Lỗ **`D6`** (chân `PB10` / RX `LPUART1` của STM32) | ESP32-C6 truyền lệnh sang STM32 |
| **GPIO 17 (`RX`)** | Lỗ **`A4`** (chân `PC1` / TX `LPUART1` của STM32) | ESP32-C6 nhận phản hồi góc nghiêng từ STM32 |

> **💡 Mẹo phần cứng cực kỳ quan trọng trên bo STM32 Nucleo-64 (`NUCLEO-L476RG`):**
> * Trên một số bo Nucleo-L476RG xuất xưởng, lỗ **D0 (`PA3`)** và **D1 (`PA2`)** trên hàng rào Arduino bị ngắt mặc định bởi 2 cầu chì hàn **`SB62` / `SB63`** ở mặt dưới bo (vì `PA2`/`PA3` đang nối vào chip nạp ST-Link).
> * Code `Project_Motion` mới đã **mở song song cả 2 cổng UART cùng lúc**:
>   1. **`USART2`** tại lỗ **`D0` (RX)** và **`D1` (TX)**.
>   2. **`LPUART1`** tại lỗ **`D6` (`PB10` - RX)** và **`A4` (`PC1` - TX)** — 2 lỗ này nối thẳng vào chân chip STM32 100% không qua cầu chì!
> * Do đó: Hãy cắm mặc định vào **`D6` (`PB10` - RX)** và **`A4` (`PC1` - TX)** trên Nucleo-L476RG để nhận lệnh ổn định ngay lập tức mà không cần hàn cầu chì `SB62`/`SB63` (`D0`/`D1` vẫn chạy song song nếu bo của bạn đã hàn cầu chì).

---

## 2. Cách nạp code cho ESP32-C6 trên Arduino IDE

1. Mở file [`ESP32_WiFi_Controller.ino`](ESP32_WiFi_Controller/ESP32_WiFi_Controller.ino).
   *(Code dùng `UART1` với `RX = GPIO 17`, `TX = GPIO 18` và WebSocket `<esp_http_server.h>` có sẵn trong ESP32 Arduino Core 3.x, không cần cài thư viện ngoài).*
2. Trong Arduino IDE:
   * Vào **Tools** $\rightarrow$ **Board** $\rightarrow$ **esp32** $\rightarrow$ chọn **`ESP32C6 Dev Module`** (hoặc `ESP32S3 Dev Module` nếu dùng S3).
   * Vào **Tools** $\rightarrow$ **Port** $\rightarrow$ chọn cổng COM của ESP32-C6.
   * Bấm **Upload**.

---

## 3. Cách nạp code `Project_Motion` cho STM32L476RG

1. Trong STM32CubeIDE, mở dự án **`Project_Motion`** (hoặc **`Project_Self_PID`** nếu chỉ thử đứng yên tại chỗ).
2. Chọn **Project $\rightarrow$ Clean...** rồi bấm **Build (biểu tượng cái búa)** và **Run / Debug** để nạp vào mạch STM32.
3. Ngay khi cấp nguồn:
   * Đặt xe nằm yên khoảng **0.5 giây** để MPU6050 tự hiệu chuẩn nhanh.
   * Đèn xanh `LD2`: **sáng đứng** = đang cân bằng (`BALANCE`), **nháy chậm** = chờ dựng xe (`FALL_WAIT`), **tắt** = chốt lỗi an toàn (`MOTOR_CUT` / `IMU_FAULT` / `WEB_ESTOP`).
   * Bấm giữ nút xanh **`B1`** trên bo STM32 để ngắt motor ngay, hiệu chuẩn lại MPU6050 (giữ nguyên mốc `DEFAULT_ZERO_DEG = -3.65f` khi `B1_CAPTURE_ZERO_ENABLE = 0`) và xóa chốt lỗi. Nút **Calib IMU (`i`)** và **Set Zero (`z`)** trên Web cũng chỉ chạy được khi xe đang dừng / nằm nghỉ.

---

## 4. Kết nối WiFi và điều khiển trên điện thoại

1. Mở WiFi điện thoại, kết nối vào mạng:
   * **Tên WiFi (SSID):** `XE_CAN_BANG_WIFI`
   * **Mật khẩu:** `12345678`
2. Mở trình duyệt (**Safari / Chrome**) truy cập địa chỉ:
   ```
   http://192.168.4.1
   ```
   Trang web tự mở kết nối **WebSocket** (`ws://192.168.4.1/ws`) giữ đúng thứ tự lệnh và hiển thị trực tiếp thông số góc nghiêng, vận tốc thực và trạng thái an toàn.
3. Các nút điều khiển trên giao diện Web:
   * **3 mức tốc độ (`1` / `2` / `3`)**: Chọn **Chậm** (`0.14 m/s`), **Vừa** (`0.22 m/s`) hoặc **Nhanh** (`0.30 m/s`).
   * **Giữ `TIẾN` (`w`) / `LÙI` (`s`)**: Gửi lệnh chạy khi chạm giữ và tự gửi lại chính lệnh `w`/`s` mỗi `150 ms` (không cộng dồn tốc độ, mất 1 byte vẫn tự khôi phục). Nhả tay gửi **`v`** để phanh dừng mượt.
   * **Giữ `TRÁI` (`a`) / `PHẢI` (`d`)**: Bẻ lái vi sai có giới hạn ưu tiên cân bằng (tự co theo dư địa PWM, tự tắt rẽ nếu sai lệch góc > $12^\circ$ và giữ nguyên vị trí tâm quay); nhả tay gửi **`c`** để trả thẳng lái.
   * **`DỪNG` (`x` / Space)**: Ngừng chạy tiến/lùi/rẽ ngay và giữ xe tự cân bằng tại chỗ.
   * **`MỞ KHÓA / ARM` (`r`)** & **`CẮT KHẨN CẤP` (`!`)**: Chốt ngắt khẩn cấp từ Web hoặc mở khóa cho xe cân bằng lại sau khi dựng đứng.
