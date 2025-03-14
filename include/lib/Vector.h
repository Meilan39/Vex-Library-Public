#ifndef VECTOR 
#define VECTOR

  #include "lib/Include.h"
  #include "lib/Helpers.h"
  
  /// @brief ベクトルを定義するクラス
  class Vector {
    public:
      float x = 0; //　ベクトルの　x　値
      float y = 0; //　ベクトルの　y　値
    public:
      /// @brief 既定のコンストラクター
      Vector();
      /// @brief x　と　y　値でベクトルを作成
      /// @param x ベクトルの　x　値
      /// @param y ベクトルの　y　値
      Vector(float x, float y) {
        this -> x = x; //
        this -> y = y; //
      }
      /// @brief 角度で単位べくとるを作成
      /// @param angle　ベクトルの角度
      Vector(float angle) {
        x = cos(angle / RadToDeg); //
        y = sin(angle / RadToDeg); //
      }
      /// @brief このベクトルとベクトル　v　の和
      /// @param v このベクトルとたすベクトル
      void add(Vector v) {
        x += v.x; //　そのまま足す
        y += v.y; //　そのまま足す
      }
      /// @brief このベクトルくの逆ベクトル ( [x,-y] -> [-x,y] )
      void invert() {
        x = -x; //　x 値を符号を変える
        y = -y; //　y 値の符号を変える
      }
      /// @brief このベクトルとベクトル　v のドット積
      /// @param v このベクトルとドット積するベクトル
      void dot(Vector v) {
        x *= v.x; //　そのまま掛ける
        y *= v.y; //　そのまま掛ける
      }
      /// @brief このベクトル掛けるスカラー　k
      /// @param k スカラーの値
      void scale(float k) {
        x *= k; //　x 値を定数　k と掛ける
        y *= k; //　y 値を定数　k と掛ける
      }
      /// @brief このベクトルを回転
      /// @param angle 時計回りの回転角度
      void rotate(float angle) {
        angle /= RadToDeg;         //　度数から弧度に変換
        float sine = sin(angle);   //　角度の正弦
        float cosine = cos(angle); //　角度の余弦
        float t = x;               //　変数　x の値を保存
        // 角度回転の定義に従い行列の乗算を行う
        x = (cosine * x) - (sine * y); //　新たな　x 値を代入
        y = (sine * t) + (cosine * y); //　新たな　y 値を代入
      }
      /// @brief　このベクトルの長さ
      /// @return 長さ
      float getMagnitude() {
        return hypot(x, y); // 斜辺を求める関数
      }
      /// @brief このベクトルの角度
      /// @return　角度（度数）
      float getAngle() {
        return atan2f(y, x) * RadToDeg; //　逆正接関数・度数に変換
      }
  };

#endif