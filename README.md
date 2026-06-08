# ESP32 FreeRTOS Smart Parking Simulation

Project ini adalah simulasi **Smart Parking Safety-Critical & Fault-Tolerant** memakai **ESP32 + FreeRTOS** di Wokwi dari dalam VSCode.

Untuk menjalankan di alat asli, lihat [HARDWARE_SETUP.md](HARDWARE_SETUP.md).


## Cara Menjalankan di VSCode

1. Install extension VSCode:
   - PlatformIO IDE
   - Wokwi Simulator
2. Buka folder ini di VSCode:
   `C:\Users\binta\OneDrive\Documents\Coding\RTOS\System_Parking`
3. Build firmware:
   - Buka PlatformIO di sidebar.
   - Pilih `esp32dev`.
   - Klik `Build`.
   - Alternatif terminal: `pio run`
4. Jalankan simulator Wokwi:
   - Tekan `F1`.
   - Pilih `Wokwi: Start Simulator`.
   - Kalau diminta license, pilih `Wokwi: Request a new License`.

Wokwi for VS Code membutuhkan file `diagram.json` dan `wokwi.toml`. Project ini sudah menyiapkan keduanya.

## Komponen Simulasi

- ESP32 DevKit
- LCD I2C 16x2
- HC-SR04 ultrasonic
- Servo gate
- LED hijau dan merah
- Buzzer
- Tombol `SLOT1`, `SLOT2`, `ENTRY`, `EXIT`, `FAULT`, `RESET`

## Pin Mapping

| Fungsi | GPIO |
|---|---:|
| Slot 1 occupied button | 32 |
| Slot 2 occupied button | 33 |
| Entry sensor button | 25 |
| Exit sensor button | 26 |
| Fault injection button | 27 |
| Reset fault button | 14 |
| Ultrasonic TRIG | 5 |
| Ultrasonic ECHO | 18 |
| Servo gate | 19 |
| Buzzer | 23 |
| LED green | 4 |
| LED red | 2 |
| LCD SDA | 21 |
| LCD SCL | 22 |

## Cara Demo

- Tekan `ENTRY` saat masih ada slot kosong: gate terbuka.
- Tekan `SLOT1` dan `SLOT2`: parkir penuh, buzzer aktif.
- Tekan `EXIT`: gate tetap bisa terbuka walaupun parkir penuh.
- Ubah jarak ultrasonic menjadi kurang dari 50 cm: gate tidak menutup karena area gerbang tidak aman.
- Tekan `FAULT`: sistem masuk fault mode, fault terkunci, buzzer aktif, dan gate fail-safe terbuka.
- Tekan `RESET`: fault dibersihkan jika tombol fault sudah dilepas dan ultrasonic valid.
- Saat boot, LED hijau dan merah akan blink bergantian sebagai self-test wiring.
- Tahan tombol sekitar 1 detik agar perubahan terlihat di Serial Monitor.

## Logic Tombol

| Tombol | Dibaca Oleh | Efek Sistem |
|---|---|---|
| `SLOT1` | `TaskSensorRead` | Slot 1 occupied, free slot berkurang |
| `SLOT2` | `TaskSensorRead` | Slot 2 occupied, jika dua slot occupied maka parking full |
| `ENTRY` | `TaskSensorRead` lalu `TaskGateControl` | Gate terbuka hanya jika parking tidak full |
| `EXIT` | `TaskSensorRead` lalu `TaskGateControl` | Gate terbuka selalu, walaupun parking full |
| `FAULT` | `TaskSensorRead` lalu `TaskSafetyMonitor` | Safety fault terkunci, buzzer aktif, gate fail-safe open |
| `RESET` | `TaskSensorRead` lalu `TaskSafetyMonitor` | Clear fault jika sensor sudah normal |

Serial Monitor akan mencetak `[BUTTON] ...` setiap ada perubahan tombol.

## Bukti RTOS untuk Dosen

Firmware memakai FreeRTOS API langsung:

- `xTaskCreatePinnedToCore()` untuk membuat task.
- `xQueueCreate()` dan `xQueueOverwrite()` untuk komunikasi data sensor.
- `xSemaphoreCreateMutex()` untuk melindungi shared state.
- `xEventGroupCreate()` untuk status event sistem.
- Priority task:
  - Safety monitor prioritas tertinggi.
  - Gate control di bawah safety.
  - Display, alarm, dan logger prioritas lebih rendah.

Serial Monitor akan menampilkan status task, core ESP32, priority, free slots, gate state, safety state, dan fault reason.

## Sumber Dokumentasi

- Wokwi ESP32: https://docs.wokwi.com/guides/esp32
- Wokwi VSCode: https://docs.wokwi.com/vscode/getting-started
- Wokwi project config: https://docs.wokwi.com/vscode/project-config
