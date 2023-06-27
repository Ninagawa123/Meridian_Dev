#include "Meridian.h"

ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_BEGIN

/**
 *  @file Meridian.cpp
 *  @brief This is a library for controlling the communication system of humanoid robots.
 *         It manages multiple servo motors and 9-axis sensors at a frequency of 100 Hz and
 *         provides real-time status information to a PC.
 *         Created by Izumi Ninagawa on October 30, 2022.
 *         MIT license.
 *         Detalis: https://github.com/Ninagawa123/Meridian_TWIN
 *  @author Izumi Ninagawa
 *  @date 2023/06/28
 *  @version 1.0.0
 *  @copyright &copy; Izumi Ninagawa & Project Meridian
 */

short Meridian::cksm_val(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    return short(~cksm);
}

/**
 * @brief Evaluate checksum of Meridim.
 *
 * @param[in] arr[] Meridim array
 * @param[in] len Length of array
 * @return true Check OK
 * @return false Check NG
 */
bool Meridian::cksm_rslt(short arr[], int len)
{
    int cksm = 0;
    for (int i = 0; i < len - 1; i++)
    {
        cksm += int(arr[i]);
    }
    if (short(~cksm) == arr[len - 1])
    {
        return true;
    }
    return false;
}

/**
 * @brief Float type value multiplied by 100 to short type.
 *
 * @param[in] val Source degree value -327.67 to 327.67
 * @return short, -32767 to 32767, Returns a value within the limit
 */
short Meridian::float2HfShort(float val)
{
    int x = round(val * 100);
    if (x > 32766)
    {
        x = 32767;
    }
    else if (x < -32766)
    {
        x = -32767;
    }
    return static_cast<short>(x);
}

/**
 * @brief Short type value divided by 100 to float type.
 *
 * @param[in] val Source value
 * @return float Source value / 100
 */
float Meridian::HfShort2float(short val)
{
    return static_cast<float>(val) / 100;
}

/**
 * @brief Kondo's KRS Servo value to degree value.
 *
 * @param[in] krs Source Kondo's KRS Servo value（3500-11500）
 * @param[in] trim Trim degree value
 * @return float, degree
 */
float Meridian::Krs2Deg(int krs, float trim)
{
    float x = ((krs - 7500 - (trim * 29.62963)) * 0.03375);
    return x;
}

/**
 * @brief Degree value to Kondo's KRS Servo value.
 *
 * @param[in] degree Source degree value
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return int, Kondo's KRS Servo value（3500-11500）
 */
int Meridian::Deg2Krs(float degree, float trim, int cw)
{
    float x = 7500 + (trim * 29.6296) + (degree * 29.6296 * cw);
    if (x > 11500) // max limit
    {
        x = 11500;
    }
    else if (x < 3500) // min limit
    {
        x = 3500;
    }
    return static_cast<int>(x);
}

/**
 * @brief Hundredfold degree value to Kondo's KRS Servo value.
 *
 * @param[in] hfdegree Source degree value * 100
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return Kondo's KRS Servo value（3500-11500）
 */
int Meridian::HfDeg2Krs(int hfdegree, float trim, int cw)
{
    float x = 7500 + (trim * 29.6296) + (hfdegree * 0.296296 * cw);
    if (x > 11500) // max limit
    {
        x = 11500;
    }
    else if (x < 3500) // min limit
    {
        x = 3500;
    }
    return static_cast<int>(x);
}

/**
 * @brief Kondo's KRS Servo value to hundredfold degree value.
 *
 * @param[in] krs Source Kondo's KRS Servo value（3500-11500）
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return int, degree * 100
 */
int Meridian::Krs2HfDeg(int krs, float trim, int cw)
{
    float x = (krs - 7500 - (trim * 29.62963)) * 3.375 * cw;
    if (x > 32766)
    {
        x = 32766;
    }
    else if (x < -32766)
    {
        x = -32766;
    }
    return static_cast<int>(x);
}

/**
 * @brief Futaba's RSxx Servo value to degree value.
 *
 * @param[in] rsxx Source Futaba's RSxx Servo value（-1600 to 1600）
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return float, degree
 */
float Meridian::RSxx2Deg(int rsxx, float trim, int cw)
{
    float x = (rsxx - (trim * 10)) * 0.1 * cw;
    return x;
}

/**
 * @brief Futaba's RSxx Servo value to hundredfold degree value.
 *
 * @param[in] rsxx Source futaba's RSxx Servo value（-1600 to 1600）
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return int, degree * 100
 */
int Meridian::RSxx2HfDeg(int rsxx, float trim, int cw)
{
    float x = (rsxx - (trim * 10)) * 10 * cw;
    if (x > 32766)
    {
        x = 32766;
    }
    else if (x < -32766)
    {
        x = -32766;
    }
    return static_cast<int>(x);
}

/**
 * @brief Degree value to Futaba's RSxx Servo value.
 *
 * @param[in] degree Source degree
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return int, Futaba's RSxx Servo value（-1600 to 1600）
 */
int Meridian::Deg2RSxx(float degree, float trim, int cw)
{
    float x = (degree + trim) * 10 * cw; //
    if (x > 1600)                        // 上限を設定
    {
        x = 1600;
    }
    else if (x < -1600) // 下限を設定
    {
        x = -1600;
    }
    return static_cast<int>(x);
}

/**
 * @brief Hundredfold degree value to Futaba's RSxx Servo value.
 *
 * @param[in] degree Source degree value * 100
 * @param[in] trim Trim degree value
 * @param[in] cw Correction value for direction of rotation（+1 or -1）
 * @return int, Futaba's RSxx Servo value（-1600 to 1600）
 */
int Meridian::HfDeg2RSxx(int degree, float trim, int cw)
{
    float x = (degree + trim) * 0.1 * cw;
    if (x > 1600) // max limit
    {
        x = 1600;
    }
    else if (x < -1600) // min limit
    {
        x = -1600;
    }
    return static_cast<int>(x);
}

/**
 * @brief Print version, I2C speed, SPI speed.
 *
 * @param[in] version This　Merisian's VERSION
 * @param[in] spi_speed SPI speed
 * @param[in] i2c_speed I2C speed
 */
void Meridian::print_tsy_hello(String version, int spi_speed, int i2c_speed)
{
    Serial.println();
    Serial.print("Hi, This is ");
    Serial.println(version);
    Serial.print("Set SPI speed: ");
    Serial.println(spi_speed);
    Serial.print("Set I2C speed: ");
    Serial.println(i2c_speed);
}

/**
 * @brief Print mounted servomotor's id.
 *
 * @param[in] idl_svmt Left side servos arrey.
 * @param[in] idr_svmt Right side servos arrey.
 */
void Meridian::print_servo_mounts(int idl_svmt[], int idr_svmt[])
{
    Serial.print("Left side Servos mounted:  ");
    for (int i = 0; i < 15; i++)
    {
        if (idl_svmt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();
    Serial.print("Right side Servos mounted: ");
    for (int i = 0; i < 15; i++)
    {
        if (idr_svmt[i])
        {
            Serial.print(i);
            Serial.print(" ");
        }
    }
    Serial.println();
}

/**
 * @brief Print mounted control pad.
 *
 * @param[in] pad_mount Type of control pad.
 * @param[in] pad_freq Freqency of calling control pad.(every ms)
 */
void Meridian::print_controlpad(int pad_mount, int pad_freq)
{
    Serial.print("Controll Pad Receiver mounted: ");
    switch (pad_mount)
    {
    case 0:
        Serial.println("None. ");
        break;
    case 1:
        Serial.print("SBDBT ");
        break;
    case 2:
        Serial.print("KRC-5FH ");
        break;
    case 3:
        Serial.print("PS3");
        break;
    case 4:
        Serial.print("PS4");
        break;
    case 5:
        Serial.print("Wii_yoko");
        break;
    case 6:
        Serial.print("Wii+Nun");
        break;
    case 7:
        Serial.print("WiiPro");
        break;
    case 8:
        Serial.print("Xbox");
        break;
    case 9:
        Serial.print("Merimote");
        break;
    case 10:
        Serial.print("Retro");
        break;
    default:
        break;
    }

    if (pad_mount != 0)
    {
        Serial.print("  freq: ");
        Serial.println(pad_freq);
    }

    delay(100);
}

/**
 * @brief Print IMU/AHRS sensor's type to serial.
 *
 * @param[in] imuahrs_mount imuahrs_mount number
 *                          0:off, 1:MPU6050(GY-521), 2:MPU9250(GY-6050/GY-9250) 3:BNO055
 * @param[in] imuahrs_freq Frequency to calling imu/ahrs via I2C.
 */
void Meridian::print_imuahrs(int imuahrs_mount, int imuahrs_freq)
{
    Serial.print("I2C IMU/AHRS Sensor mounted: ");

    switch (imuahrs_mount)
    {
    case 0:
        Serial.print("None. ");
        break;
    case 1:
        Serial.print("MPU6050(GY-521) ");
        break;
    case 2:
        Serial.print("MPU9250(GY-6050/GY-9250) ");
        break;
    case 3:
        Serial.print("BNO055 ");
        break;
    default:
        break;
    }

    if (imuahrs_mount != 0)
    {
        Serial.print("  freq: ");
        Serial.println(imuahrs_freq);
    }
}

/**
 * @brief Increase sequence number from imput. (0 to 59,999)
 *
 * @param[in] previous_seq_num Previous sequence number.
 * @return int, Expected sequence number.
 */
int Meridian::increase_seq_num(int previous_seq_num)
{
    int x = previous_seq_num + 1;
    if (x > 59999) // Reset counter
    {
        x = 0;
    }
    return x;
}

/**
 * @brief Generate expected sequence number from imput.(0 to 59,999)
 *
 * @param[in] previous_seq_num Previous sequence number.
 * @return int, Expected sequence number.
 */
int Meridian::predict_seq_num(int previous_seq_num)
{
    int x = previous_seq_num + 1;
    if (x > 59999) // 予想値が59,999以上ならカウントを0に戻す
    {
        x = 0;
    }
    return x;
}

/**
 * @brief Compare expected seq number and received seq number.(0 to 59,000)
 *
 * @param[in] predict_seq_num Previous sequence number.
 * @param[in] received_seq_num Received sequence number.
 * @return true OK
 * @return false NG
 */
bool Meridian::compare_seq_nums(int predict_seq_num, int received_seq_num)
{
    return (predict_seq_num == received_seq_num);
}

/**
 * @brief Display gamepad keys.
 *
 * @param[in] arr array[3]
 */
void monitor_joypad(uint16_t *arr)
{
    for (int i = 0; i < 4; i++)
    {
        Serial.print(arr[i]);
        if (i < 3)
        {
            Serial.print("/");
        }
    }
    Serial.println();
}

/**
 * @brief Show text massage if monitor_flow is true. This is for debagging.
 *
 * @param[in] text Text message to display.
 * @param[in] monitor_flow True:on ,False;off
 */
void Meridian::monitor_check_flow(const String &text, bool monitor_flow)
{
    if (monitor_flow)
    {
        Serial.print(text);
    }
}

/**
 * @brief Show servo error id.
 *
 * @param[in] text String. "L","R","C",etc
 * @param[in] num Servo id.
 * @param[in] monitor_servo_error True:on ,False;off
 */
void Meridian::monitor_servo_error(const String &text, int num, bool monitor_servo_error)
{
    if (monitor_servo_error)
    {
        Serial.print("Servo err ");
        Serial.print(text);
        Serial.print("_");
        Serial.println(num);
    }
}

/**
 * @brief Print wake-up massage to serial.
 *
 * @param[in] version VESRION of Meridian.
 * @param[in] serial_pc_bps PC-Serial speed.
 * @param[in] wifi_ap_ssid Wi-Fi AP adress to connect.
 */
void Meridian::print_esp_hello_start(String version, String serial_pc_bps, String wifi_ap_ssid)
{
    Serial.println();
    Serial.println("Hello, This is " + version); // バージョン表示
    delay(100);
    Serial.println("PC Serial Speed : " + serial_pc_bps + " bps"); // PCシリアル速度
    Serial.println("WiFi connecting to => " + wifi_ap_ssid);       // WiFi接続完了通知
}

/**
 * @brief Print Wi-Fi status to serial.
 *
 * @param[in] wifi_send_ip Send Wi-Fi IP. (PC's IP adress)
 * @param[in] wifi_localip Wi-Fi IP of this device.
 * @param[in] fixed_ip_addr Fixed Wi-Fi IP of this device. (If mode_fixed_ip is True)
 * @param[in] mode_fixed_ip Use fixed Wi-Fi IP or Not.
 */
void Meridian::print_esp_hello_ip(String wifi_send_ip, IPAddress wifi_localip, String fixed_ip_addr, bool mode_fixed_ip)
{
    Serial.println("WiFi successfully connected.");                          // WiFi接続完了通知
    Serial.println("PC's IP address target is  => " + String(wifi_send_ip)); // 送信先PCのIPアドレスの表示

    if (mode_fixed_ip)
    {
        Serial.println("ESP32's IP address is  => " + String(fixed_ip_addr) + " (*Fixed)"); // ESP32自身のIPアドレスの表示
    }
    else
    {
        Serial.print("ESP32's IP address is  => "); // ESP32自身のIPアドレスの表示
        Serial.println(wifi_localip);
    }
}

ARDUINO_ROBOTICS_ESPTEENSY_MERIDIAN_NAMESPACE_END
