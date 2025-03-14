#ifndef VELOCITY_PROFILE 
#define VELOCITY_PROFILE

  #include "lib/Include.h"

  /// @brief 速度プロフィールを定義するクラス
  class StaticProfile {
    private:
      float s1; //初期速度 (0 から 1)
      float s2; //最終速度 (0 から 1)
      float k1; //加速 (0　以上)
      float k2; //減速 (0　以上)
      float m;  //最大速度 (0 から 1)
      float d;  //現在値と目的値の差
    public:
      /// @brief 速度プロフィールのコンストラクター
      /// @param initial_velocity 初期速度 (0 から 1)
      /// @param final_velocity 最終速度 (0 から 1)
      /// @param acceleration_slope 加速 (0　以上)
      /// @param deceleration_slope 減速 (0　以上)
      /// @param maximum_velocity 最大速度 (0 から 1)
      /// @param distance 現在値と目的値の差
      StaticProfile(float initial_velocity, float final_velocity, float acceleration_slope, float deceleration_slope, float maximum_velocity, float distance = 100) {
          this -> s1 = initial_velocity;
          this -> s2 = final_velocity;
          this -> k1 = acceleration_slope;
          this -> k2 = deceleration_slope;
          this -> m = maximum_velocity;
          this -> d = distance;
      }
      /// @brief 現在値に相応しい速度出力を返します 
      /// 下記の式も自作でシグモイド関数に基づく
      /// m^2 / ( (1 + (m / s1 - 1) * e^(-k1 * x) ) * (1 + (m / s2 - 1) * e^(k2 * x - k2 * d) ) )
      /// @param current システムの現在値
      /// @return 速度出力 (0 から 1)
      float get(float current) {
          float c1 = m / s1 - 1;
          float c2 = m / s2 - 1;
          c1 = c1 * pow(E, -k1 * current) + 1;
          c2 = c2 * pow(E, k2 * current - k2 * d) + 1;
          return ( m * m ) / ( c1 * c2 );
      }
  };
  
  #endif