# Phelan Butterfly V1 — ESP32 Flapping-Wing Flight Controller

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--C3-blue.svg)](#)
[![Arduino IDE](https://img.shields.io/badge/Arduino%20IDE-2.x-teal.svg)](#)

> Hệ thống điều khiển mô hình bướm vỗ cánh (Ornithopter) 2 servo dùng vi điều khiển ESP32 / ESP32-C3, tích hợp giao diện điều khiển mặt đất **Tactical Web Deck** qua Wi-Fi thời gian thực.

---

## Mục Lục

- [Điểm Nổi Bật](#điểm-nổi-bật)
- [Cấu Trúc Dự Án](#cấu-trúc-dự-án)
- [Pinout](#pinout)
- [Yêu Cầu Phần Cứng](#yêu-cầu-phần-cứng)
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
| **Plug & Play** | Mở trình duyệt bất kỳ (Chrome / Safari trên iOS & Android) — không cần cài App |
| **SoftAP Wi-Fi** | Cắm nguồn pin là tự phát Wi-Fi riêng, không cần router |
| **Anti-Brownout** | Vô hiệu hóa bộ phát hiện sụt áp sớm (`disable_bod_early`), giải quyết triệt để lỗi reset ESP32 khi 2 servo vỗ ăn dòng tức thời |
| **Throttle Interlock** | Bắt buộc hạ tay ga về 0% mới cho phép ARM, chống quạt gãy cánh ngoài ý muốn |
| **Failsafe 500ms** | Mất kết nối Wi-Fi/Web quá 500ms — tự ngắt ga + nhả tải servo |
| **Emergency STOP** | Chạm 1 chạm ngắt toàn bộ hệ thống |
| **Throttle Hold** | Cần ga giữ nguyên vị trí, cần lái tự động hồi tâm khi nhả tay |
| **PWM 300Hz** | Servo phản hồi siêu tốc, thuật toán sóng tam giác LUT 256 bước cho vỗ cánh tự nhiên |

---

## Cấu Trúc Dự Án

```
buomv1/
├── buomv1.ino      Firmware ESP32 (Flight Controller chính)
├── index.html      Giao diện Web Deck (Tactical Flight Deck)
├── server.js       Simulator WebSocket trên PC (dev/test)
├── .gitignore
├── LICENSE
└── README.md
```

---

## Pinout

| Linh Kiện | GPIO | Chức Năng |
| :--- | :---: | :--- |
| Servo Cánh Trái (L) | `5` | Vỗ cánh + lái vi sai cánh trái (PWM 300Hz) |
| Servo Cánh Phải (R) | `6` | Vỗ cánh + lái vi sai cánh phải (PWM 300Hz) |
| LED Trạng Thái | `8` | Chớp chậm: Chờ Wi-Fi · Chớp nhanh: DISARMED · Sáng: ARMED |

---

## Yêu Cầu Phần Cứng

| Thành phần | Gợi ý |
| :--- | :--- |
| Vi điều khiển | ESP32 DevKit hoặc ESP32-C3 Mini |
| Servo (x2) | Micro servo 3.7g – 9g, tốc độ cao |
| Nguồn pin | LiPo 2S (7.4V) |
| BEC | Mạch hạ áp 5V / 3A – 5A |
| Khung cánh | Khung bướm Phelan 80cm (xem file PDF trong repo) |

---

## Thư Viện Phần Mềm

Cài đặt trong Arduino IDE, Library Manager:

1. [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) — Kevin Harrington, John K. Bennett
2. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) + [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)

Board package: **ESP32 by Espressif** (>= 2.0.0) trong Boards Manager.

---

## Hướng Dẫn Sử Dụng

### 1. Nạp Firmware

1. Mở `buomv1.ino` bằng Arduino IDE 2.x.
2. Chọn Board: `ESP32 Dev Module` hoặc `ESP32-C3 Dev Module`.
3. Sửa cấu hình Wi-Fi:
   ```cpp
   constexpr char WIFI_STA_SSID[] = "TenWiFiCuaBan";
   constexpr char WIFI_STA_PASS[] = "MatKhauCuaBan";
   ```
   Để trống SSID `""` sẽ luôn chạy chế độ SoftAP (tự phát Wi-Fi riêng).
4. Compile & Upload.

### 2. Kết Nối

1. Cấp nguồn cho bướm. LED nhấp nháy khi đang kết nối Wi-Fi.
2. LED sáng đều khi đã kết nối thành công.
3. Mở Serial Monitor (115200 baud) để xem IP được cấp.

### 3. Điều Khiển

1. Mở trình duyệt, truy cập `http://phelan.local` hoặc `http://<IP_ESP32>`.
2. Xoay ngang màn hình điện thoại để vào Tactical Flight Deck.
3. Đảm bảo cần ga ở 0%, nhấn **ARM** để mở khóa servo.

| Cần | Lên / Xuống | Trái / Phải |
| :--- | :--- | :--- |
| Trái | Throttle (ga vỗ cánh) | Yaw (lái hướng) |
| Phải | Pitch (chúi/ngửa) | Roll (nghiêng) |

Nhấn nút đỏ **STOP** để ngắt khẩn cấp bất cứ lúc nào.

---

## Tùy Chỉnh Thông Số

Mở `buomv1.ino`, namespace `Config`:

| Tham số | Mặc định | Mô tả |
| :--- | :---: | :--- |
| `FLAP_AMP` | `65` | Biên độ góc vỗ cánh cực đại (deg) |
| `FLAP_SPEED_MIN` / `MAX` | `157286` / `262144` | Dải tốc độ vỗ theo tay ga |
| `SERVO_HZ` | `300` | Tần số PWM servo |
| `REVERSE_SERVO_L` / `R` | `false` | Đảo chiều servo nếu lắp ngược cơ khí |
| `WIFI_STA_SSID` | `""` | Tên Wi-Fi (trống = SoftAP) |
| `FAILSAFE_DIHEDRAL` | `15` | Góc mở cánh khi failsafe (deg) |

---

## Simulator

Dùng `server.js` để giả lập ESP32 WebSocket trên máy tính — test giao diện Web Deck mà không cần phần cứng.

```bash
node server.js
# Mở trình duyệt: http://localhost:8080
```

Yêu cầu Node.js >= 16. Simulator mô phỏng giao thức WebSocket binary, telemetry packet và heartbeat PING/PONG giống ESP32 thật.

---

## Đóng Góp

1. Fork repo này
2. Tạo branch mới: `git checkout -b feature/ten-tinh-nang`
3. Commit thay đổi: `git commit -m "feat: mo ta ngan"`
4. Push branch: `git push origin feature/ten-tinh-nang`
5. Mở Pull Request

Phát hiện lỗi? Tạo [Issue](../../issues) kèm mô tả chi tiết.

---

## Giấy Phép

[MIT License](LICENSE)
