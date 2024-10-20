# STL Reorienter  
STLの軸や中心を再設定できるpythonです。  

<img width="400" src="https://github.com/user-attachments/assets/0732bd7a-2c48-4147-bbb5-7af2c1a22864">
  
## 機能  
・読み込んだSTLファイルのオブジェクトを0,0,0を原点として表示する。  
・STLの0,0,0を再設定できる。  
・座標軸を再設定できる。  
（画面に見えている状態の上をzプラス、右をyプラスとして保存できる。）  
  
## ライブラリ等のバージョン  
- python Version: 3.11
- numpy Version: 2.1.1
- PyQt5 Version: 5.15.11
- pvtk　Version: 9.3.1
にて動作を確認しました。  

## conda での環境構築

```  
$ conda create -n urdf python=3.11  
$ conda activate urdf  
$ conda install numpy=2.1.1  
$ pip install PyQt5==5.15.11  
$ pip install vtk==9.3.1  
```

他の方法だと仮想環境にqt6がインストールされる場合があるのでご注意ください。  
  
## 実行方法  
```
cd [urdf_reorienter.pyがあるディレクトリ]  
python urdf_reorienter.py  
```  
  
## 操作方法  
as,ws,qeキー：座標軸を上下、左右、ロールに90度ずつ回転させる  
rキー：カメラをリセットする  
tキー：オブジェクト表示の透過の切り替え  
カーソルキー；新しい原点となる紫ポインタの移動（画面に対して上下左右に10mmずつ移動）  
シフト＋カーソル：1ミリずつ移動  
コマンド(Ctrl)＋カーソル：0.1ミリずつ移動  
Ctrl + c ：終了  
マウスドラッグ：回転  
シフト＋マウスドラッグ：並行移動  
SETボタン：インプットフィールドに入力した値の場所にポインタをセットする
RESETボタン：紫ポインタを原点にリセットする
Load STL File：STLファイルを読みこむ  
Save as reoriented STL：STLを紫ポインタを原点として保存する
  
好きな方向にオブジェクトを回転させたり、原点を移動させたりしてから保存してください。  
画面に対して上下をz軸、左右y軸として保存するので、  
必ずrキーでリセットした後、as,ws,qeキーで90度ずつ回転させてから保存してください。  
