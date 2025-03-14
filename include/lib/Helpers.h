#ifndef HELPERS 
#define HELPERS

  #include "lib/Include.h"

  /// @brief 二つの角度の最短角度差を返す関数
  /// @param current 現在角度
  /// @param desired 目的角度
  /// @return 二つの角度の最短角度差
  float wrap(float current, float desired) {  
    float error = current - desired; // 現在と目的値の差
    // 差の絶対値が180より多ければ逆方向へと回る
    if (error > 180) error -= 360; 
    else if (error < -180) error += 360;
    return -error;
  }

  /// @brief 角度を０度から360度の間に制限する
  /// @param angle 制限する角度
  /// @return 0度から360度に制限された同じ角度
  float bound( float angle ) {
    return fmod( fmod(angle, 360) + 360, 360 );
  }
  
  /// @brief 数字の配列の平均を取る
  /// @param nums 数字だけの　std::vector
  /// @return 配列の平均
  float avg(std::vector<float> nums) {
    if (nums.empty()) return 0; // ０で割らないよう
    float total = 0;
    for (float n : nums) total += n; // 要素の和
    return total / nums.size();      // 和を要素の数で割る
  }

  /// @brief 値を最低限と最高限の間に制限する
  /// @param value 制限する値
  /// @param min 最低限
  /// @param max 最高限
  /// @return 制限された値
  float fitToRange(float value, float min, float max) {
    if (value > max) return max; // 最高限より高いー最高限を返す
    if (value < min) return min; // 最低限より低いー最低限を返す
    return value;                // 範囲内の場合ーそのまま返す
  }

  /// @brief 値が閾値の範囲内にあるか確認
  /// @param value 確認する値
  /// @param setpoint 閾値
  /// @param threshold 認める範囲
  /// @return 値が範囲内か否か
  bool inRange(float value, float setpoint, float threshold) {
    return (setpoint - threshold < value) && (value < setpoint + threshold);
  }

#endif