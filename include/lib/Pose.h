#ifndef POSE 
#define POSE

  #include "lib/Include.h"
  #include "lib/Helpers.h"
  #include "lib/Vector.h"

  /// @brief ロボット姿勢を表すクラス
  class Pose {
    public:
      float x; //　ロボットの位置の　x 値
      float y; //　ロボットの位置の　y 値
      float w; //　ロボットの角度（度数法）
    public:
      /// @brief ロボットの位置だけ定義する
      /// @param v ロボットの位置を表すベクトル
      void setVector(Vector v) {
        x = v.x;
        y = v.y;
      }
      /// @brief 姿勢　”pose” とこの姿勢の差
      /// @param pose 差を取る姿勢
      /// @return 二つの姿勢の差を表す姿勢オブジェクト
      Pose getError(Pose pose) {
        Pose error;
        error.y = pose.y - y;
        error.x = pose.x - x;
        error.w = wrap(w, pose.w); //　二つの角度の最短角度差を返す関数
        return error;
      }
      /// @brief 姿勢の位置情報だけ返す
      /// @return ロボットの位置を表すベクトル
      Vector getVector() {
        return Vector {x, y};
      }
  };

#endif