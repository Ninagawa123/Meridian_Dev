
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;

// meridian_20220604_unity_ParamMaster.cs
// 各オブジェクトから送られてきたデータを格納する。
// 画面上のフィギアはこのデータを元に表示する

public class ParamMaster : MonoBehaviour
{
    //meridimの値
    public static short[] r_meridim = new short[90];//meridim90の受信用
    public static short[] s_meridim = new short[90];//meridim90の送信用
    public static short[] s_meridim_udp = new short[90];//meridim90の送信用(UDP送信転記用)
    public static bool s_meridim_udp_write_flag = false; //s_meridim_udpが書き込み中（なので読み取り禁止）というフラグ
    public static bool s_meridim_udp_read_flag = false; //s_meridim_udpが読み込み中（なので書き込み禁止）というフラグ
    public static bool s_udp_queue = false; //s_meridim_udpの制御キュー（転記完了で+1(1)、送信完了で-1(0)）

    //meridimのインスペクター表示用
    public short[] r_meridim_disp = new short[90];//meridim90の受信用
    public short[] s_meridim_disp = new short[90];//meridim90の送信用

    //サーボの現在位置（最終結果）：サーボの初期補正値 + (サーボの移動差分 * サーボの回転補正値)
    public static float[] ServoAngles_L = new float[20];
    public static float[] ServoAngles_R = new float[20];

    //ServoAngles_L,Rのインスペクター表示用：サーボの現在位置
    public float[] ServoAngles_L_disp = new float[20];
    public float[] ServoAngles_R_disp = new float[20];

    //UnityUIからの入力を一旦格納用
    public static float[] ServoAngles_L_UI = new float[20];
    public static float[] ServoAngles_R_UI = new float[20];

    //サーボの初期位置補正用
    public static float[] ServoAngles_L_init = new float[20];
    public static float[] ServoAngles_R_init = new float[20];

    //サーボ回転方向の正負方向補正用(pn:positive,negative)
    public static float[] ServoAngles_L_pn = new float[20];
    public static float[] ServoAngles_R_pn = new float[20];

    //サーボコマンド受信用
    public static int[] ServoCommand_L_r = new int[15];
    public static int[] ServoCommand_R_r = new int[15];

    //サーボコマンド送信用
    public static int[] ServoCommand_L_s = new int[15];
    public static int[] ServoCommand_R_s = new int[15];

    //サーボの移動差分
    public static float[] ServoAngles_L_diff = new float[20];
    public static float[] ServoAngles_R_diff = new float[20];

    //サーボの前回位置の値
    public static float[] ServoAngles_L_past = new float[20];
    public static float[] ServoAngles_R_past = new float[20];

    //計算モーション用のサーボ位置
    public static float[] ServoAngles_L_motion = new float[20];
    public static float[] ServoAngles_R_motion = new float[20];

    //計算モーション用の変数
    public float motion_count = 0;//計算モーションのカウント用
    public float motion_test_val = 0;//計算モーション用のテンポラリ変数

    //センサー用の配列　それぞれ100倍された値が入ってくる
    public static float[] ACxyz_GYxyz_CPxyz_TP_RPY = new float[13];//11,12,13番がRPYの100倍値

    //UI上のスイッチ状態の判定用
    public static bool actionToggle = false;//計算制御テスト用トグル
    public bool actionToggle_disp = false;//計算制御テスト用トグルのインスペクター表示用

    public static bool sendToggle = false;//計算制御テスト用トグル
    public bool sendToggle_disp = false;//計算制御テスト用トグルのインスペクター表示用

    //コマンド格納用の変数
    public static int mastercommand = 90;//meridim90の送信用

    // フレームスキップ検出用のシーケンシャルカウンタ (-30,000~30,000) meridim[01]番 // ★★★
    public static int frame_sync_s = -30000; // ★★★




    private void Start()
    {
        //サーボ用変数のリセット
        for (int i = 0; i < 20; i++)
        {
            ServoAngles_L[i] = 0;
            ServoAngles_R[i] = 0;
        }

        //サーボの初期位置補正（degreeで指定）
        //※ハード側の補正値はTeensyで指定。こちらはUnity側の計算および表示での補正用。
        ServoAngles_L_init[0] = 0;
        ServoAngles_L_init[1] = 0;
        ServoAngles_L_init[2] = 0;
        ServoAngles_L_init[3] = 0;
        ServoAngles_L_init[4] = 0;
        ServoAngles_L_init[5] = 0;
        ServoAngles_L_init[6] = 0;
        ServoAngles_L_init[7] = 0;
        ServoAngles_L_init[8] = 0;
        ServoAngles_L_init[9] = 0;
        ServoAngles_L_init[10] = 0;
        ServoAngles_R_init[0] = 0;
        ServoAngles_R_init[1] = 0;
        ServoAngles_R_init[2] = 0;
        ServoAngles_R_init[3] = 0;
        ServoAngles_R_init[4] = 0;
        ServoAngles_R_init[5] = 0;
        ServoAngles_R_init[6] = 0;
        ServoAngles_R_init[7] = 0;
        ServoAngles_R_init[8] = 0;
        ServoAngles_R_init[9] = 0;
        ServoAngles_R_init[10] = 0;

        //サーボ回転方向の正負方向補正（正なら1,負なら-1）
        //※ハード側の補正値はTeensyで指定。こちらはUnity側の計算および表示での補正用。
        ServoAngles_L_pn[0] = 1;
        ServoAngles_L_pn[1] = 1;
        ServoAngles_L_pn[2] = 1;
        ServoAngles_L_pn[3] = 1;
        ServoAngles_L_pn[4] = 1;
        ServoAngles_L_pn[5] = 1;
        ServoAngles_L_pn[6] = 1;
        ServoAngles_L_pn[7] = 1;
        ServoAngles_L_pn[8] = 1;
        ServoAngles_L_pn[9] = 1;
        ServoAngles_L_pn[10] = 1;
        ServoAngles_R_pn[0] = 1;
        ServoAngles_R_pn[1] = 1;
        ServoAngles_R_pn[2] = 1;
        ServoAngles_R_pn[3] = 1;
        ServoAngles_R_pn[4] = 1;
        ServoAngles_R_pn[5] = 1;
        ServoAngles_R_pn[6] = 1;
        ServoAngles_R_pn[7] = 1;
        ServoAngles_R_pn[8] = 1;
        ServoAngles_R_pn[9] = 1;
        ServoAngles_R_pn[10] = 1;
    }


    void FixedUpdate()//パラムマスターの計算
    {

        //:::::::: 計算で動かす場合の数値を毎フレーム作成（デモ用モーション） ::::::::::::::::::::::::::::::::::::::::::::::::

        //サーボを計算で動かすときのテスト用サインカーブの種
        motion_test_val = Mathf.Sin(motion_count) * 20 - 20;

        //計算モーション用の元の数値の作成。しゃがんだり立ったりを繰り返す
        ServoAngles_L_motion[0] = motion_test_val;
        ServoAngles_L_motion[1] = 0;
        ServoAngles_L_motion[2] = motion_test_val + 40;
        ServoAngles_L_motion[3] = motion_test_val + 50;
        ServoAngles_L_motion[4] = motion_test_val;
        ServoAngles_L_motion[5] = 0;
        ServoAngles_L_motion[6] = 0;
        ServoAngles_L_motion[7] = motion_test_val;
        ServoAngles_L_motion[8] = -motion_test_val * 2;//膝だけ逆方向に2倍動く
        ServoAngles_L_motion[9] = motion_test_val;
        ServoAngles_L_motion[10] = motion_test_val;//0
        ServoAngles_L_motion[0] = 0;
        ServoAngles_R_motion[1] = 0;
        ServoAngles_R_motion[2] = motion_test_val + 40;
        ServoAngles_R_motion[3] = motion_test_val + 50;
        ServoAngles_R_motion[4] = motion_test_val;
        ServoAngles_R_motion[5] = 0;
        ServoAngles_R_motion[6] = 0;
        ServoAngles_R_motion[7] = motion_test_val;
        ServoAngles_R_motion[8] = -motion_test_val * 2;//膝だけ逆方向に2倍動く
        ServoAngles_R_motion[9] = motion_test_val;
        ServoAngles_R_motion[10] = motion_test_val;//0
        motion_count += 0.01f;

        //:::::::: コマンドやトグルに応じてサーボ位置の受信反映/制御送受に振り分け ::::::::::::::::::::::::::::::::::::::::::::::::

        for (int i = 0; i < 15; i++)
        {
            //まずサーボ位置の受信結果をLRともいったん代入
            ServoAngles_L[i] = ServoAngles_L_init[i] + (ServoAngles_L_diff[i] * ServoAngles_L_pn[i]);
            ServoAngles_R[i] = ServoAngles_R_init[i] + (ServoAngles_R_diff[i] * ServoAngles_R_pn[i]);

            //:::::::: L系統の処理 ::::::::
            if (ServoCommand_L_s[i] == 0) //サーボのコマンドが0ならサーボ実機は脱力なので受信位置データを画面に反映
            {
                if (Mathf.Abs(ServoAngles_L[i] - ServoAngles_L_past[i]) > 0.8)//誤差が0.8より大なら反映（表示プルプルの防止）
                {
                    //サーボの現在位置 = サーボの初期補正値 + (サーボの移動差分*サーボの回転補正値)
                    ServoAngles_L[i] = ServoAngles_L_init[i] + (ServoAngles_L_diff[i] * ServoAngles_L_pn[i]);
                    ServoAngles_L_past[i] = ServoAngles_L[i];//現在のサーボ位置を次回用の過去位置としてキープ
                }
                else
                {
                    ServoAngles_L[i] = ServoAngles_L_past[i];//移動差分が0.8未満なら位置は前回と同じにする
                }
            }
            else //サーボのコマンドが0以外ならサーボ実機は電源オンなので位置データを送信
            {
                if (actionToggle)//UIのアクショントグルがオンなら、計算したモーションデータをサーボの移動差分データとする
                {
                    ServoAngles_L_diff[i] = ServoAngles_L_motion[i];
                }
                else//UIのアクショントグルがオフなら、スライダーのデータをサーボの移動差分データとする
                {
                    ServoAngles_L_diff[i] = ServoAngles_L_UI[i];
                }
                ServoAngles_L_past[i] = ServoAngles_L_diff[i];//現在のサーボ位置を次回用の過去位置としてキープ
            }

            //:::::::: R系統の処理 ::::::::
            if (ServoCommand_R_s[i] == 0)//サーボのコマンドが0ならサーボ実機は脱力なので受信位置データを画面に反映

            {
                if (Mathf.Abs(ServoAngles_R[i] - ServoAngles_R_past[i]) > 0.8)//誤差が0.8より大なら反映（表示プルプルの防止）
                {
                    //サーボの現在位置 = サーボの初期補正値 + (サーボの移動差分*サーボの回転補正値)
                    ServoAngles_R[i] = ServoAngles_R_init[i] + (ServoAngles_R_diff[i] * ServoAngles_R_pn[i]);
                    ServoAngles_R_past[i] = ServoAngles_R[i];//現在のサーボ位置を次回用の過去位置としてキープ
                }
                else
                {
                    ServoAngles_R[i] = ServoAngles_R_past[i];//移動差分が0.8未満なら位置は前回と同じにする
                }
            }
            else //サーボのコマンドが0以外ならサーボ実機は電源オンなので位置データを送信
            {
                if (actionToggle)//UIのアクショントグルがオンなら、計算したモーションデータをサーボの移動差分データとする
                {
                    ServoAngles_R_diff[i] = ServoAngles_R_motion[i];
                }
                else//UIのアクショントグルがオフなら、スライダーのデータをサーボの移動差分データとする
                {
                    ServoAngles_R_diff[i] = ServoAngles_R_UI[i];
                }
                ServoAngles_R_past[i] = ServoAngles_R_diff[i];//現在のサーボ位置を次回用の過去位置としてキープ
            }

            //:::::::: mastercommandの処理 ::::::::
            if (mastercommand == 10002)
            {
                s_meridim[0] = 10002;//#IMUのヨー軸センターリセットコマンド
                mastercommand = 90;//90はマスターコマンドのデフォルト値
            }

        }

        //トグルがオンならサーボデータの計算結果をs_meridimへ反映
        if (actionToggle | sendToggle)
        {
            for (int i = 0; i < 15; i++)
            {
                s_meridim[i * 2 + 21] = (short)Mathf.Round(ServoAngles_L[i] * 100);
                s_meridim[i * 2 + 51] = (short)Mathf.Round(ServoAngles_R[i] * 100);
            }
        }

        // SPI送信タイミングの「待ち受けモード」の指定 // ★★★
        s_meridim[0] = (short)5; //5:Teensy主導, 6:外部待ち受け // ★★★

        // @フレームスキップ検出用のカウントをカウントアップして送信用に格納 // ★★★
        frame_sync_s++;           // シーケンシャルカウンタを加算 // ★★★
        if (frame_sync_s > 29999) // 予想値が29,999以上ならカウントを-30000に戻す // ★★★
        { // ★★★
            frame_sync_s = -30000; // ★★★
        } // ★★★
        s_meridim[1] = (short)frame_sync_s; // ★★★

        //:::::::: s_meridimの作成 ::::::::

        while (s_meridim_udp_read_flag)//s_meridim_udpが読み取りアクセス中なら終了まで待つ
        {
            Thread.Sleep(1);
        }

        if (s_udp_queue == false)
        {

            s_meridim_udp_write_flag = true;//trueならs_meridim_udpが書き込み中（なので読み取り禁止）というフラグ
            for (int i = 0; i < 90; i++)//meridimをインスペクター表示用に転記
            {
                s_meridim_udp[i] = s_meridim[i];
            }

            s_meridim_udp_write_flag = false;//s_meridim_udpの書き込み中フラグを下げる
            s_udp_queue = true; //s_meridim_udpの制御キュー（転記完了で+1(1)、送信完了で-1(0)）
        }

        for (int i = 0; i < 90; i++)//meridimをインスペクター表示用に転記
        {
            r_meridim_disp[i] = r_meridim[i];
            s_meridim_disp[i] = s_meridim[i];
        }
        for (int i = 0; i < 15; i++)//ServoAngles_LRをインスペクター表示用に転記
        {
            ServoAngles_L_disp[i] = ServoAngles_L[i];
            ServoAngles_R_disp[i] = ServoAngles_R[i];
        }

    }

    private void Update()
    {
        //UIトグルのオンオフ結果をインスペクターにも表示する
        sendToggle_disp = sendToggle;//スライダ値送信用トグル
        actionToggle_disp = actionToggle;//計算制御テスト用のトグル
    }
}
