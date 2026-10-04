# Phelan Butterfly V1 — ESP32 Flapping-Wing Flight Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--C3-blue.svg)](#)
[![Arduino IDE](https://img.shields.io/badge/Arduino%20IDE-2.x-teal.svg)](#)

> Hệ thống điều khiển mô hình bướm vỗ cánh (Ornithopter) 2 servo dùng vi điều khiển ESP32 / ESP32-C3, trang bị cặp servo kỹ thuật số kim loại **PTK 7452** và hệ thống nguồn **Pin LiPo 2S (7.4V)** qua mạch hạ áp **BEC 5V**, tích hợp giao diện mặt đất **Tactical Web Deck** qua Wi-Fi thời gian thực.

---

## Mục Lục

- [Điểm Nổi Bật](#điểm-nổi-bật)
- [Cấu Trúc Dự Án](#cấu-trúc-dự-án)
- [Cấu Hình Phần Cứng & Nguồn Điện](#cấu-hình-phần-cứng--nguồn-điện)
- [Sơ Đồ Kết Nối (Pinout & Wiring)](#sơ-đồ-kết-nối-pinout--wiring)
- [Thư Viện Phần Mềm](#thư-viện-phần-mềm)
- [Hướng Dẫn Sử Dụng](#hướng-dẫn-sử-dụng)
- [Tùy Chỉnh Thông Số](#tùy-chỉnh-thông-số)
- [Simulator](#simulator)
- [Đóng Góp](#đóng-góp)
- [Giấy Phép](#giấy-phép)

---

## Điểm Nổi Bật

| Tính năng | Mô tả |
| :--- | :--- |
| **Cặp Servo PTK 7452** | Điều khiển trực tiếp 2 servo kỹ thuật số kim loại PTK 7452 ở tần số xung 300Hz, lực kéo khỏe, chịu tải lực cản cánh và chống rơ nhông khi vỗ liên tục |
| **Hệ Thống Nguồn 2S + BEC** | Sử dụng pin LiPo 2S (7.4V - 8.4V) phối hợp mạch hạ áp BEC 5V (3A - 5A) độc lập, cung cấp dòng xả tức thời đủ lớn cho 2 servo mà không gây sụt áp vi điều khiển |
| **Plug & Play** | Mở trình duyệt bất kỳ (Chrome / Safari trên iOS & Android) — không cần cài App |
| **SoftAP Wi-Fi** | Cấp nguồn pin là tự phát Wi-Fi riêng, không cần router |
| **Anti-Brownout** | Vô hiệu hóa bộ phát hiện sụt áp sớm (`disable_bod_early`), kết hợp cấu hình nguồn BEC giải quyết triệt để lỗi reset ESP32 khi 2 servo ăn dòng đỉnh |
| **Throttle Interlock** | Bắt buộc hạ cần ga về 0% mới cho phép ARM, chống quạt gãy sườn hoặc gãy cánh ngoài ý muốn |
| **Failsafe 500ms** | Mất kết nối Wi-Fi/Web quá 500ms — tự động ngắt ga và đưa servo về góc lượn an toàn (`FAILSAFE_DIHEDRAL`) |
| **Emergency STOP** | Chạm 1 nút dừng toàn bộ hệ thống ngay lập tức |
| **Throttle Hold** | Cần ga giữ nguyên vị trí, cần lái hướng (Yaw, Pitch, Roll) tự động hồi tâm khi nhả tay |

---

## Cấu Trúc Dự Án

```
buomv1/
├── buomv1.ino                                       Firmware ESP32 (Flight Controller chính)
├── index.html                                       Giao diện Web Deck (Tactical Flight Deck)
├── server.js                                        Simulator WebSocket trên PC (dev/test)
├── Dimension_A1_print_Phelan_butterfly_ 80cm.pdf    Bản vẽ thiết kế & kích thước khung cánh 80cm
├── .gitignore
├── LICENSE
└── README.md
```

---

## Cấu Hình Phần Cứng & Nguồn Điện

### 1. Danh Sách Linh Kiện

| Thành phần | Quy cách khuyến nghị | Ghi chú kỹ thuật |
| :--- | :--- | :--- |
| **Vi điều khiển** | ESP32 Dev Module hoặc ESP32-C3 Dev Module | Xung nhịp 160MHz - 240MHz, tích hợp Wi-Fi + mDNS |
| **Servo (x2)** | **PTK 7452** (hoặc PTK 7452 MG-D) | Digital Metal Gear, xung điều khiển 300Hz, điện áp 4.8V - 6.0V |
| **Nguồn pin** | **LiPo 2S 7.4V** (300mAh – 650mAh, 30C - 75C) | Dòng xả cao, trọng lượng nhẹ, đảm bảo công suất vỗ cánh |
| **Mạch hạ áp (BEC)** | **BEC rời 5V / 3A – 5A** (hoặc Step-Down UBEC) | Bắt buộc: hạ áp từ 2S (7.4V) xuống 5V ổn định nuôi cặp servo PTK 7452 |
| **Khung cánh** | Khung bướm Phelan sải cánh 80cm | Kèm bản vẽ in khổ A1 (`Dimension_A1_print_Phelan_butterfly_ 80cm.pdf`) |

### 2. Nguyên Lý Cấp Nguồn An Toàn (Power Architecture)

Để đảm bảo ESP32 không bị sập nguồn khi 2 servo PTK 7452 đồng thời đổi chiều ở tần số 300Hz:
1. **Nguồn Servo**: Chân `VCC (+)` của 2 servo PTK 7452 lấy trực tiếp từ đầu ra **5V của mạch BEC**. Tuyệt đối **không** lấy nguồn servo từ chân `5V/VIN` của kit ESP32.
2. **Nguồn ESP32**: Đầu ra 5V của BEC đồng thời cấp vào chân `VIN` (hoặc `5V`) của ESP32.
3. **Chung Mass (Common GND)**: Nối chung toàn bộ cực âm `GND` của Pin 2S, BEC, Kit ESP32 và 2 servo PTK 7452.

---

## Sơ Đồ Kết Nối (Pinout & Wiring)

| Linh Kiện | Chân Tín Hiệu | Chân Nguồn | Chức Năng |
| :--- | :---: | :---: | :--- |
| **Servo PTK 7452 Trái (L)** | `GPIO 5` | BEC 5V / GND | Động lực vỗ cánh + lái vi sai cánh trái (PWM 300Hz) |
| **Servo PTK 7452 Phải (R)** | `GPIO 6` | BEC 5V / GND | Động lực vỗ cánh + lái vi sai cánh phải (PWM 300Hz) |
| **LED Trạng Thái** | `GPIO 8` | Onboard / GND | Chớp chậm: Đang kết nối · Chớp nhanh: DISARMED · Sáng: ARMED |
| **Nguồn Pin 2S (7.4V)** | — | Vào BEC IN (+/-) | Cung cấp toàn bộ công suất hoạt động cho hệ thống |

---

## Thư Viện Phần Mềm

Cài đặt thông qua Library Manager trên Arduino IDE:

1. [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) — Kevin Harrington, John K. Bennett
2. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) + [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)

Board package yêu cầu: **ESP32 by Espressif** (>= 2.0.0) trong Boards Manager.

---

## Hướng Dẫn Sử Dụng

### 1. Nạp Firmware

1. Mở file [buomv1.ino](file:///d:/buomv1/buomv1.ino) bằng Arduino IDE 2.x.
2. Chọn Board: `ESP32 Dev Module` hoặc `ESP32-C3 Dev Module`.
3. Cấu hình thông tin Wi-Fi trong namespace `Config`:
   ```cpp
   // Điền SSID & mật khẩu nếu muốn bướm kết nối vào mạng Wi-Fi có sẵn:
   constexpr char WIFI_STA_SSID[] = "YourWiFiSSID";
   constexpr char WIFI_STA_PASS[] = "YourWiFiPassword";
   ```
   *Ghi chú: Để trống `""` sẽ tự động chạy hoàn toàn ở chế độ SoftAP (tự phát mạng riêng).*
4. Biên dịch và nạp (Compile & Upload).

### 2. Kết Nối Thiết Bị

1. Cắm giắc pin LiPo 2S vào mạch BEC. Đèn LED trên vi điều khiển nhấp nháy báo hiệu khởi động.
2. Khi hệ thống sẵn sàng, kết nối điện thoại vào mạng Wi-Fi do bướm phát hoặc mạng chung.
3. Mở Serial Monitor (baud rate 115200) để kiểm tra IP hoặc dùng mDNS.

### 3. Điều Khiển Bay

1. Mở trình duyệt web (Safari trên iOS hoặc Chrome trên Android), truy cập:
   * **`http://phelan.local`** (hoặc địa chỉ IP hiển thị trên Serial Monitor).
2. Xoay ngang màn hình thiết bị để kích hoạt giao diện Tactical Flight Deck.
3. Kéo cần ga về vị trí `0%`, sau đó bấm nút **ARM** để kích hoạt lực điều khiển servo.
4. Thao tác điều khiển:
   * **Cần Trái**: Đẩy lên để tăng ga vỗ cánh (Throttle), gạt trái/phải để chuyển hướng (Yaw).
   * **Cần Phải**: Gạt lên/xuống để Chúi/Ngửa (Pitch), gạt trái/phải để Nghiêng (Roll).
   * **Nút STOP**: Ngắt toàn bộ hoạt động khẩn cấp khi gặp sự cố.

---

## Tùy Chỉnh Thông Số

Mở [buomv1.ino](file:///d:/buomv1/buomv1.ino), hiệu chỉnh tại namespace `Config`:

| Tham số | Giá trị chuẩn PTK 7452 | Đơn vị | Ý nghĩa kỹ thuật |
| :--- | :---: | :---: | :--- |
| `SERVO_HZ` | `300` | Hz | Tần số PWM điều khiển servo kỹ thuật số PTK 7452 |
| `SERVO_CENTER` | `90` | deg | Vị trí góc trung lập cơ khí của servo |
| `SERVO_MIN_ANGLE` | `20` | deg | Giới hạn góc quét cơ học dưới |
| `SERVO_MAX_ANGLE` | `160` | deg | Giới hạn góc quét cơ học trên |
| `FLAP_AMP` | `65` | deg | Biên độ góc quét vỗ cánh cực đại khi kéo ga tối đa |
| `FLAP_SPEED_MIN` | `157286` | — | Bước tăng chu kỳ vỗ cánh ở mức ga thấp nhất |
| `FLAP_SPEED_MAX` | `262144` | — | Bước tăng chu kỳ vỗ cánh ở mức ga cao nhất |
| `FAILSAFE_DIHEDRAL` | `15` | deg | Góc nâng cánh chữ V tự lượn khi mất tín hiệu điều khiển |
| `REVERSE_SERVO_L` | `false` | bool | Đảo chiều servo trái nếu lắp ngược càng truyền động |
| `REVERSE_SERVO_R` | `false` | bool | Đảo chiều servo phải nếu lắp ngược càng truyền động |

---

## Simulator

Dự án cung cấp file [server.js](file:///d:/buomv1/server.js) để giả lập trạm phát ESP32 trên máy tính, phục vụ kiểm thử giao diện Tactical Web Deck mà không cần cấp nguồn phần cứng thật:

```bash
node server.js
# Mở trình duyệt truy cập: http://localhost:8080
```

Yêu cầu môi trường Node.js >= 16. Server mô phỏng đầy đủ giao thức nhị phân WebSocket, truyền nhận telemetry và phản hồi gói tin Heartbeat PING/PONG.

---

## Đóng Góp

1. Fork kho lưu trữ này.
2. Tạo nhánh tính năng: `git checkout -b feature/ten-tinh-nang`.
3. Lưu commit: `git commit -m "feat: mo ta ngan ve tinh nang"`.
4. Đẩy nhánh: `git push origin feature/ten-tinh-nang`.
5. Tạo Pull Request để xem xét tích hợp.

Mọi vấn đề phát sinh hoặc đề xuất cải tiến xin vui lòng gửi qua mục [Issues](../../issues).

---

## Giấy Phép

Phát hành theo giấy phép [MIT License](LICENSE) — Bản quyền (c) 2026 PhelanDev.
