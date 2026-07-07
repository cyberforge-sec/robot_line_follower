# robot_line_follower

Arduino-based line follower using 6 TCRT5000 sensors and PID control for autonomous path tracking

🤖 Robot Line Follower (Arduino + IR Sensor)

Proyek ini adalah robot line follower berbasis Arduino Uno dan sensor TCRT5000, yang menggunakan algoritma PID untuk mengikuti garis berwarna hitam pada latar putih. Cocok untuk lomba robotik dasar dan edukasi.

🖼️ Gambar Lintasan

![alt text](https://github.com/cyberforge-sec/robot_line_follower/blob/main/ard/Gambar_Lintasan.png?raw=true)

🔧 Spesifikasi Hardware & Software
Lihat detail lengkap pada file [spesifikasi.txt](ard/code/spesifikasi.txt)

🧠 Fitur Utama
- 6 sensor IR
- Kontrol kecepatan PWM
- Deteksi kehilangan garis
- Deteksi persimpangan
- Belokan tajam dan zigzag

🛠️ Komponen
- Arduino Uno R3
- TCRT5000 (6 buah)
- Motor Driver L298N
- 2 Motor DC
- Caster ball
- Baterai 7.4V
- Sasis plastik

🚀 Cara Instalasi
1. Download dan install [Arduino IDE](https://www.arduino.cc/en/software)
2. Buka file `ard/code/line_follower.ino`
3. Sambungkan Arduino Uno ke komputer
4. Upload sketch ke board
5. Pastikan sensor IR terhubung ke pin A0-A5 dan motor ke L298N

📐 Kalibrasi
Sebelum digunakan, pastikan garis hitam dikenali oleh sensor (bisa lewat Serial Monitor jika ingin debug). Ubah nilai `Kp`, `Ki`, dan `Kd` sesuai performa di lintasan nyata.

⚙️ Konfigurasi Default PID

Kp = 25;
Ki = 0;
Kd = 15;

🧪 Tips Pemakaian
- Gunakan latar putih dengan garis hitam tebal 18–25 mm.
- Hindari cahaya langsung ke sensor.
- Gunakan baterai penuh untuk hasil maksimal.
