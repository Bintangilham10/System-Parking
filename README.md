# System Parking - ESP32 FreeRTOS Simulation

System Parking adalah simulasi smart parking berbasis **ESP32**, **FreeRTOS**, **PlatformIO**, dan **Wokwi**. Project ini dibuat untuk mendemonstrasikan sistem parkir dengan kontrol gerbang otomatis, monitoring slot, alarm parkir penuh, dan mekanisme safety fault-tolerant.

## Fitur Utama

- Monitoring 2 slot parkir menggunakan tombol simulasi.
- Kontrol gate otomatis menggunakan servo.
- Sensor entry dan exit untuk membuka gerbang.
- Deteksi area gerbang menggunakan sensor ultrasonic HC-SR04.
- Mode parkir penuh dengan indikator LED dan buzzer.
- Fault injection manual untuk menguji skenario kegagalan sistem.
- Fail-safe behavior: gate tetap terbuka saat fault atau area gerbang tidak aman.
- LCD I2C 16x2 untuk menampilkan slot kosong, status gate, jarak, dan fault.
- Serial Monitor untuk melihat log state sistem secara real time.

## Teknologi

| Kebutuhan | Teknologi |
|---|---|
| Board | ESP32 DevKit |
| Framework | Arduino |
| RTOS | FreeRTOS |
| Build system | PlatformIO |
| Simulator | Wokwi for VSCode |
| Bahasa | C++ |

## Struktur Project

```text
.
|-- .vscode/
|   |-- extensions.json
|   |-- launch.json
|   `-- settings.json
|-- src/
|   `-- main.cpp
|-- diagram.json
|-- platformio.ini
|-- wokwi.toml
|-- .gitignore
`-- README.md
```

## Komponen Simulasi

- ESP32 DevKit
- LCD I2C 16x2
- HC-SR04 ultrasonic sensor
- Servo motor sebagai gate
- LED hijau dan LED merah
- Buzzer
- Tombol `SLOT1`, `SLOT2`, `ENTRY`, `EXIT`, `FAULT`, dan `RESET`

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

## Cara Menjalankan

1. Install extension VSCode:
   - PlatformIO IDE
   - Wokwi Simulator
2. Buka folder project di VSCode.
3. Build firmware:

   ```powershell
   pio run
   ```

   Jika perintah `pio` belum tersedia, gunakan:

   ```powershell
   python -m platformio run
   ```

4. Jalankan simulator Wokwi:
   - Tekan `F1`.
   - Pilih `Wokwi: Start Simulator`.
   - Jika diminta license, pilih `Wokwi: Request a new License`.

Wokwi menggunakan `diagram.json` untuk rangkaian dan `wokwi.toml` untuk menunjuk hasil build firmware:

```toml
firmware = ".pio/build/esp32dev/firmware.bin"
elf = ".pio/build/esp32dev/firmware.elf"
```

## Skenario Demo

| Aksi | Hasil |
|---|---|
| Tekan `ENTRY` saat slot masih tersedia | Gate terbuka |
| Tekan `SLOT1` | Slot kosong berkurang |
| Tekan `SLOT1` dan `SLOT2` | Parkir penuh, buzzer aktif |
| Tekan `EXIT` saat parkir penuh | Gate tetap dapat terbuka |
| Atur jarak ultrasonic kurang dari 50 cm | Gate tidak menutup karena area gerbang belum aman |
| Tekan `FAULT` | Sistem masuk fault mode, buzzer aktif, gate fail-safe terbuka |
| Lepas `FAULT`, lalu tekan `RESET` | Fault dibersihkan jika sensor ultrasonic valid |

Saat boot, LED hijau dan merah akan blink bergantian sebagai self-test wiring. Tahan tombol sekitar 1 detik agar perubahan mudah terlihat di Serial Monitor.

## Logika Tombol

| Tombol | Dibaca Oleh | Efek Sistem |
|---|---|---|
| `SLOT1` | `TaskSensorRead` | Slot 1 dianggap terisi |
| `SLOT2` | `TaskSensorRead` | Slot 2 dianggap terisi |
| `ENTRY` | `TaskSensorRead`, `TaskGateControl` | Gate terbuka jika parkir belum penuh |
| `EXIT` | `TaskSensorRead`, `TaskGateControl` | Gate terbuka walaupun parkir penuh |
| `FAULT` | `TaskSensorRead`, `TaskSafetyMonitor` | Fault terkunci, buzzer aktif, gate fail-safe terbuka |
| `RESET` | `TaskSensorRead`, `TaskSafetyMonitor` | Membersihkan fault jika kondisi sensor sudah normal |

Serial Monitor akan mencetak log `[BUTTON]` saat input berubah dan log `[STATE]` setiap 1 detik.

## Implementasi FreeRTOS

Firmware menggunakan beberapa task yang dipisahkan berdasarkan tanggung jawab:

| Task | Prioritas | Core | Tanggung Jawab |
|---|---:|---:|---|
| `SensorRead` | 3 | 0 | Membaca tombol dan ultrasonic, lalu mengirim snapshot sensor |
| `SafetyMonitor` | 5 | 1 | Mengevaluasi fault, blocked area, dan status safety |
| `GateControl` | 4 | 1 | Mengontrol servo gate berdasarkan entry, exit, full, dan safety |
| `Alarm` | 2 | 1 | Mengontrol buzzer dan LED |
| `Display` | 2 | 0 | Menampilkan status sistem ke LCD |
| `Logger` | 1 | 0 | Menulis status sistem ke Serial Monitor |

RTOS primitive yang digunakan:

- `xTaskCreatePinnedToCore()` untuk membuat task dan menentukan core.
- `xQueueCreate()` dan `xQueueOverwrite()` untuk komunikasi data sensor terbaru.
- `xSemaphoreCreateMutex()` untuk melindungi shared system state.
- `xEventGroupCreate()` untuk menyimpan event status sistem.
- `vTaskDelay()` untuk scheduling periodik tanpa blocking loop utama.

## Safety Behavior

Sistem dirancang dengan prinsip fail-safe:

- Jika sensor ultrasonic gagal membaca jarak, sistem masuk fault mode.
- Jika task sensor tidak mengirim data dalam batas waktu tertentu, sistem masuk fault mode.
- Jika area gate masih terhalang, gate dibiarkan terbuka.
- Jika fault aktif, gate terbuka dan buzzer menyala.
- Fault manual akan tetap terkunci sampai tombol `FAULT` dilepas dan `RESET` ditekan.

## Sumber Referensi

- Wokwi ESP32: https://docs.wokwi.com/guides/esp32
- Wokwi for VSCode: https://docs.wokwi.com/vscode/getting-started
- Wokwi project configuration: https://docs.wokwi.com/vscode/project-config
- PlatformIO ESP32: https://docs.platformio.org/en/latest/platforms/espressif32.html
