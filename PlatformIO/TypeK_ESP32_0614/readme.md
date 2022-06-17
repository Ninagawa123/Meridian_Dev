## PlatformIOへの導入方法

### PlatformIO でプロジェクトを作成する
「Meridian_TypeK_ESP32_0614」とする。
フレームワークはArduinoを選ぶ。
  
### ボードの選択
「Boads」メニューより検索で「Espressif 32」を選択。
  
### ESP32のPlatformsのバージョン
「Platforms」メニューより「Espressif 32」のバージョン「3.5.0」を選択。  
※ 3.5.0はどういうわけか以降のアップデートに比べてwifiの性能がよい。（新しいものは送受信時にデータにノイズが出る）
  
### 「ESP32DMASPI」の導入
アリマーク→QUICK ACCESS→ PIO Home → Open → Libraries の検索窓で「ESP32DMASPI」  
「ESP32DMASPI by Hideaki Tai」を選び、バージョンは0.1.2とする（※0.2.0では動かない）
「Add to Project」でプロジェクトを選択し「Add」ボタンで導入。
  
### PS4リモコンの導入
libのインポートなどにルールがあり、下記にまとめました。  
またPS4ライブラリをESP32用に修正する方法もまとめています。  
https://qiita.com/Ninagawa_Izumi/items/d8966092fe2abd3cba79
  
### wiimoteの導入
https://github.com/hrgraf/ESP32Wiimote  
を前述のPS4リモコンと同様に導入するが、階層構造が違うので変更する。  
解凍してできた「ESP32Wiimote-master」をlibに入れる。  
「ESP32Wiimote-master」の中に「src」というディレクトリを作り、   
ESP32Wiimote.cpp, ESP32Wiimote.h, TinyWiimote.cpp, TinyWiimote.h  
をいれる。
  
### platformio.iniの変更
platformio.iniをアップしてあるものに差し替える。
   
## PlatformIOとArduinoIDEの違い・注意点
ArduinoIDEでxxxx.inoというスケッチのファイル名は、PlatformIOではmain.cppとなる。  
main.cppの冒頭に必ず #include <Arduino.h> が必要。  
Vscodeを使う時など、変数や関数の利用は宣言よりも後ろの行に書かないとエラーを吐く。  
