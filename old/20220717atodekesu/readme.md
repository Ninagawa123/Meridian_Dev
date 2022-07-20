Meridian Board Type.K 用のプログラムです。

ESP32 → main.cpp をplatformIOで書き込みます。33行目〜で　アクセスポイントやIPアドレスの設定が必要です。PS4リモコンの接続がデフォルトになっています。
Teensy4.0 → Meridian_TWIN_for_Teensy_20220717a.ino を Teensyduino IDE で書き込みます。IMUセンサはMPU6050の設定になっています。
Meridian_console_v220717a.py → windowsのpythonで開けます。61行目〜でIPアドレスの設定が必要です。

進捗：
ほぼ完璧に動くようになりました。
通常の利用時はほぼエラーなくwifi&spi通信が成立します。実用的。
PS4リモコンを利用するとまだ若干エラーが出ます。調整でもう少しよくなるかも。PS4リモコンとKRC-5FHの同時接続も可能です。
コードをもう少し整理し、新しいリポジトリにてMeridian_TWINとして公開予定です。
