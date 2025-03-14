#ifndef PIDCLASS 
#define PIDCLASS

  #include "lib/Include.h"
  #include "lib/Helpers.h"

  /// @brief PID制御を簡単に扱えるクラス
  class PID {
    private:
      float p; // Pゲイン
      float i; // Iゲイン
      float d; // Dゲイン
      float f; // Fゲイン
      float lastTime;  //　前ループの時間
      float lastError; //　前ループの偏差
      float accum;     //　累積額
      // 初期化
      bool init = true; //　初期値
      // 範囲関係
      bool range = false; // 範囲に制限
      float min; // 最低限
      float max; // 最高限
    public:
        /// @brief PID 制御器を作成
        /// @param p Pゲイン
        /// @param i Iゲイン
        /// @param d Dゲイン
        /// @param f Fゲイン
        /// @param min 最低値
        /// @param max 最高値
        PID(float p = 0, float i = 0, float d = 0, float f = 0, float min = 0, float max = 0) {
            this -> p = p;
            this -> i = i;
            this -> d = d;
            this -> f = f;
            // 範囲に入らない場合制限
            if (min != 0 || max != 0) {
                this -> max = max;
                this -> min = min;
                this -> range = true;
            }
        }
        /// @brief PID制御の出力を得る
        /// @param position 現在値
        /// @param setpoint 目的値
        /// @return PID制御の出力
        float get(float position, float setpoint) {
            float error = setpoint - position; // 偏差を求める
            float time = vex::timer::system(); // 現在時間を記録
            // 求められたら初期化
            if (init) {
              lastTime = time - 1; // ０で割らないよう
              lastError = error;   // 前回偏差を初期化
              accum = 0;           // 累積額を０に
              init = false;        // 初期対策を繰り返さないよう
            }
            // PID制御の公式に従い適切な操作量を求める
            accum = (error * (time - lastTime) + accum); // 偏差の積分を近似する
            float kp = p * error; // 比例制御
            float ki = i * accum; // 残留偏差
            float kd = d * ( (error - lastError) / (time - lastTime) ); // 修正
            float kf = copysignf(f, error); // 定出力
            lastTime = time;   // 次回ループに備える
            lastError = error; // 次回ループに備える
            if (!range) return kp + ki + kd + kf; // 範囲の制限がなければそのまま返す
            else return fitToRange(kp + ki + kd + kf, min, max); // 制限があれば制限を行う
        }
        /// @brief 制御を初期化
        void reset() {
            init = true;
        }
  };


#endif