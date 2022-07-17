Meridian Board Type.K 用のプログラムです。

ESP32 → main.cpp をplatformIOで書き込みます。33行目〜で　アクセスポイントやIPアドレスの設定が必要です。PS4リモコンの接続がデフォルトになっています。
Teensy4.0 → Meridian_TWIN_for_Teensy_20220717a.ino を Teensyduino IDE で書き込みます。IMUセンサはMPU6050の設定になっています。
Meridian_console_v220717a.py → windowsのpythonで開けます。61行目〜でIPアドレスの設定が必要です。

課題：
ESP32 が 同じUDPデータを何度も受け取るように振る舞うようです。結果、シーケンシャルカウンタ（-30000〜+29999までカウント、meridim配列[1]に格納）のチェックが変になります。
ESP32 はPCで生成されたカウントは読むだけで同じ数字をそのままTeensyに流しています。
