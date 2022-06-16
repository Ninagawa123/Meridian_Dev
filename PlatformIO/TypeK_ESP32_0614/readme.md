### PlatformIO でプロジェクトを作成する
「Meridian_TypeK_ESP32_0614」とする。
  
### ボードの選択
「Boads」メニューより検索で「Espressif 32」を選択。
  
### ESP32のPlatformsのバージョン
「Platforms」メニューより「Espressif 32」のバージョン「3.5.0」を選択。  
※ 3.5.0は以降のアップデートに比べてwifiの性能がよい。
  
### 「ESP32DMASPI」の導入
アリマーク→QUICK ACCESS→ PIO Home → Open → Libraries の検索窓で「ESP32DMASPI」  
「ESP32DMASPI by Hideaki Tai」を選び、「Add to Project」  
プロジェクトを選択し「Add」ボタンで導入。
  
### PS4リモコンの導入
libのインポートなどにルールがあり、下記にまとめました。  
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
   
