# PWM AC Chopper STM32 + GUI

## Pin yang digunakan

- PWM utama: `PA8` (`TIM1_CH1`)
- PWM complementary: `PA7` (`TIM1_CH1N`)
- ADC pressure sensor: `PC0` (`ADC1_IN1`)
- Serial: `USART1`, TX `PA9`, RX `PA10`, baudrate `115200`

## Sensor pressure

Sensor dianggap memiliki karakteristik:

- Rentang tekanan: `0` sampai `12 bar`
- Output sensor: `0.5` sampai `4.5 V`
- Divider sebelum ADC: `10k` dan `20k`
- Rasio yang dipakai program: `20k / (10k + 20k) = 0.6667`

Program menghitung:

1. Tegangan ADC pada pin STM32.
2. Tegangan asli sensor sebelum divider.
3. Tekanan dalam bar.

## Format telemetry dari STM32

STM32 mengirim data tiap `100 ms`. Semua nilai pecahan dikirim sebagai integer
fixed-point agar tidak bergantung pada dukungan `printf` float di firmware:

```text
DATA,<ms>,<adc_raw>,<adc_mv>,<sensor_mv>,<pressure_bar_x1000>,<duty_percent_x100>,<frequency_hz>,<estimated_rms_x100>
```

Contoh:

```text
DATA,12345,2048,1650,2475,5925,5000,5000,11000
```

Pada contoh di atas, GUI akan menampilkan `1.650 V`, `2.475 V`, `5.925 bar`,
`50.00 %`, dan `110.00 Vrms`.

## Command dari GUI/laptop ke STM32

```text
GET
SET,DUTY,<0-95>
SET,FREQ,<100-50000>
SET,BOTH,<0-95>,<100-50000>
STOP
```

STM32 akan membalas dengan `OK,...` atau `ERR,...`.

## Menjalankan GUI Python

Install dependency:

```powershell
python -m pip install -r requirements.txt
```

Jalankan GUI:

```powershell
python pwm_ac_chopper_gui.py
```

Di GUI:

1. Pilih port serial STM32.
2. Klik `Connect`.
3. Atur duty cycle dan frequency.
4. Klik `Apply Both`.
5. Aktifkan `CSV Logging` jika ingin menyimpan data eksperimen.

## Catatan keselamatan

Rangkaian `220 VAC` berbahaya. Gunakan isolasi yang benar, fuse, enclosure,
driver gate optocoupler/isolated driver, dan alat ukur yang sesuai. Pastikan
ground laptop/USB tidak terhubung langsung ke rangkaian yang referensinya ikut
jaringan listrik.
