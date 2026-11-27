# Motor PID Panel v4.2 — Project_self_tuning (ICM-20602, VM ổn áp 6.0V)

Chạy `Motor_PID_Panel.exe` trên Windows. Đóng PuTTY / Serial Monitor trước, chọn cổng COM của ST-LINK rồi bấm **Kết nối**.

## Tính năng & bộ thông số nền (v4.2 — Khung CAD `Khung_Frame.step` + VM `6.0V` + Encoder `5V`)
- Tính chuẩn từ hình học `Khung_Frame.step` (`M ≈ 0.35 kg`, `l_com = 0.033 m`, `vệt bánh = 150 mm`) và bộ ổn áp cấp motor **`6.0V`**:
  - Vòng trong: `PID = 92.00 / 0.00 / 5.50` (`Ki = 0`, PD thuần).
  - Vòng ngoài: `PPID = 9.00 / 1.80 / 12.50`.
  - Khóa hướng Yaw: `YPID = 0.30 / 1.20`.
  - Vùng chết mềm hộp số N20: `DEAD = 110`, `RAMP = 48` (kèm ICM-20602 DLPF Gyro `92 Hz` / Accel `44.8 Hz`, `I2C1 100 kHz + DNF=10` và fixed gyro bias đo tĩnh `12000` mẫu trong firmware).
- Nút **Preset Êm / Preset Chuẩn / Preset Cứng** trên thanh công cụ để đổi nhanh cấu hình cho nguồn `6.0V`.
- Nút **`-` / `+`** chỉnh nhanh trực tiếp cho `Kp`, `Kd`, `Kp_p`, `Kd_p`, `ZERO`, `DEAD`, và hỗ trợ **nhấn `Enter`** ngay trong ô số để gửi lệnh.
- Lưu ý phần cứng: khi cấp **`5V`** cho Hall Encoder GA12-N20 trên chân `TIM2` (`PA0/PA1`) & `TIM3` (`PA6/PA7`), hãy chắc chắn dùng chân **`FT` (5V-tolerant)** và điện trở kéo hợp lý (`Nucleo-L476RG` có `PA0/PA1/PA6/PA7` đều là chân `FT` chịu 5V, nhưng không gắn thêm `FT_a`/analog).

## Build & kiểm tra tự động
```powershell
.\build.ps1
.\Motor_PID_Panel.exe --self-test .\SELF_TEST.txt
.\Motor_PID_Panel.exe --ui-test .\UI_TEST.txt
.\Motor_PID_Panel.exe --render-preview .\preview.png
```
