# Maridian_Base
(まだ開発中なのでドキュメントも日本語で進めます.）

メリディアンはヒューマノイドロボットの状態情報をデバイス間で循環させ、
ロボットに搭載した制御マイコンとPCとで常に最新の状態情報を共有するシステムです。
PC画面のシミュレーション結果と実際のロボットの動作を高速通信でリアルタイムに連動させることができます。
UnityやROSとの連携に対応しています。

# Demo
このデモでは、ROSのRvisを利用して、脱力状態のサーボロボットの関節を手で動かすと、画面内のロボットにその動きが即時反映されるということを楽しめます。

# Requirement
使用する機材は以下の5つです。

* 1.サーボロボット(KHR-4HV等)

* 2.Teensy4.0 (サーボ制御用)

* 3.ESP32devkitC (wifi通信用)

* 4.Meridian Boar もしくはブレッドボード

* 5.PC

# Features
デバイス間の連携は以下の経路での双方向通信となります。

サーボモーター-(半二重シリアル)-Teensy4.0-(SPI)-ESP32-(Wifi/UDP通信)-(アクセスポイント)-PC/Unity/ROS

メリディアンのプロトロコルはMeridim配列と命名したUDP通信１パケットに収まるコンパクトな書式です。
約90個のShort型のデータ配列で、30個分のサーボデータに加え、IMU情報やユーザーが作成したデータも盛り込むことができます。

# Installation

まず、Meridian Boardを準備します。回路図の草案はpdfに同梱していますので、ブレッドボード等で作成することができます。
（今後、プリント基板を同人ハードとして頒布することも予定しています。）

次にMeridian_Base_for_Teensy40.inoとMeridian_Base_for_ESP32.inoのファイルをそれぞれArduinoIDE、Teensyduino等を使ってTeensy4.0,ESP32に書き込みます。
最初に必要なライブラリの導入が必要です。

###Teensy4.0用として必要なライブラリ

* TsyDMASPI (Teensyduinoの「ツール」→「ライブラリマネージャ」より検索＆インストール)

* MPU6050 (同上)

* Madgwick (同上)

* IcsHardSerialClass(https://kondo-robot.com/faq/ics-library-a2 より「ICS_Library_for_Arduino_V2.1」をDLし、
　Teensyduinoの「スケッチ」→「ライブラリをインクルード」→「.ZIP形式のライブラリをインストール...」
　
* MPU6050_6Axis_MotionApps20(https://github.com/carloschar/MPU6050_6Axis5 より「MPU6050_6Axis_MotionApps20.h」をDLし、
　I2CDevのフォルダをTeensyduinoのlibrariesディレクトリに移して再起動。

###ESP32用として必要なライブラリ
* ESP32DAMSPI(Teensyduinoの「ツール」→「ライブラリマネージャ」より検索＆インストール)

最後に、手元の環境にROSパッケージを作成し、rosnode_meridim_base.pyを含めてください。

https://github.com/Ninagawa123/roid1_urdf

を実行してRvisで立ち上げた上、
先ほど作成したパッケージのrosnode_meridim_base.pyを実行してください。
ロボットの関節を手で動かした時の姿勢が、Rvis上のロボット画像にも反映されます。

ちょっと説明不足すぎる気もしますが、説明文も今後開発を進めながら追記していきます。
