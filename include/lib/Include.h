#ifndef INCLUDE 
#define INCLUDE

  #include "vex.h"
  #include <vector>
  #include <string>

  /* ロボットのポートID */

  const int FR_id = vex::PORT20; //　右前モータ
  const int FL_id = vex::PORT19; //　左前モータ
  const int RL_id = vex::PORT13; //　左後ろモータ
  const int RR_id = vex::PORT11; //　右後ろモータ
  const int inertial_id = vex::PORT12;     //　イナーシャルセンサ
  const int encoderRight_id = vex::PORT14; //　右の車輪を図るエンコーダー
  const int encoderLeft_id = vex::PORT15;  //　左の車輪を図るエンコーダー
  const int encoderRear_id = vex::PORT10;  //　後ろの車輪を図るエンコーダー

  /* 数学定数 */

  const float PI = 3.14159265359; //　piの値
  const float E = 2.71828182846;  //  Eの値
  const float SMALL = 0.00001;    //  小さい値

  /* 変換 */

  const float RadToDeg = 180 / PI;  //　掛けて弧度法を度数法に・割って度数法を弧度法

  /* その他 */

  const int BAND = 3;
  const float autonomous_rotation_scaler = 0.4; //　自動操作特有の回転スカラー


#endif