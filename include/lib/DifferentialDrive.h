#ifndef DIFFERENTIALDRIVE
#define DIFFERENTIALDRIVE

  #include "lib/Include.h"
  #include "lib/Vector.h"
  #include "lib/Pose.h"
  #include "lib/Trajectory.h"
  #include "lib/PID.h"

  /// @brief 一般的非ホロノミック系ロボットの車台クラス
  class DifferentialDrive {
    private:
      vex::motor FR {FR_id, ratio18_1, false}; // 右前のモータを定義
      vex::motor FL {FL_id, ratio18_1, true }; // 左前のモータを定義
      vex::motor RL {RL_id, ratio18_1, true }; // 左後ろのモータを定義
      vex::motor RR {RR_id, ratio18_1, false}; // 右後ろのモータを定義
      vex::inertial inertial {inertial_id, vex::turnType::left}; // イナーシャルセンサの定義
      vex::rotation encoderRight {encoderRight_id}; // 右の車輪に付いているエンコーダー
      vex::rotation encoderLeft {encoderLeft_id};   // 左の車輪に付いているエンコーダー
    private:
      const float MAX_VELOCITY = 200; // 最高速度の定数（rpm）
      const float W_SCALER = 0.6; // 回転スカラー（比例的ー０から１）
      const float ODOMETRY_WHEEL_DIAMEMETER = 2.75; // 車輪の直径
      const float DpsToRps = ODOMETRY_WHEEL_DIAMEMETER * PI / 360; // dps（度毎秒）と　ips（インチ毎分）の変更用
      const float asyncDriveSpeed = 0.12; // 非同期運転速度
      float lastTime = 0; // 前ループ記録した時間
      float distanceTraveled = 0; // 走った距離
      PID omegaPID {0.008, 0, 0, 0.008, -1, 1}; // PID制御クラスの定義
    public:
      Pose pose {0, 0, 0};    // ロボットの姿勢オブジェクトを宣言
      Vector velocity {0, 0}; // ロボットの速度オブジェクトを宣言
    private:
      /// @brief 右と左車輪の出力を独立することでロボットを実際に操れる関数
      /// @param right 右車輪の出力 (-1から1)
      /// @param left 左車輪の出力 (-1から1)
      void drive(float left, float right) {
        right *= MAX_VELOCITY; // 適当の速度を一般出力から導く
        left *= MAX_VELOCITY;  //　適当の速度を一般出力から導く
        FR.spin(forward, right, rpm); //　右前の出力を命令する
        FL.spin(forward, left , rpm); //　左前の出力を命令する
        RR.spin(forward, right, rpm); //　右後ろの出力を命令する
        RL.spin(forward, left , rpm); //　左後ろの出力を命令する
      }
      /// @brief ロボットの角度をイナーシャルセンサに問う
      /// @return ロボットの角度（度数）
      float getGyro() {
        return inertial.heading(); 
      }
      /// @brief イナーシャルセンサの角度を変更
      /// @param heading 角度（度数）
      void setGyro( float heading ) {
        inertial.setHeading(heading, vex::rotationUnits::deg);
      }
    public:    
      /// @brief 車台の初期化
      void init() {
        inertial.startCalibration(); // イナーシャルセンサの初期化
        FR.setBrake(brake); // 右前のモータをブレークモードに設定
        FL.setBrake(brake); // 左前のモータをブレークモードに設定
        RR.setBrake(brake); // 右後ろのモータをブレークモードに設定
        RL.setBrake(brake); // 左後ろのモータをブレークモードに設定
        encoderLeft.setReversed(false); // 左エンコーダーの方向を設定
        encoderRight.setReversed(true); // 右エンコーダーの方向を設定
        reset(); // 経路関係の変数の初期化
        while(inertial.isCalibrating()) {wait(100, msec);} // センサの初期化処理を待つ
      }
      /// @brief 経路実行前に変数の初期化
      void reset() {
        distanceTraveled = 0; // 走った距離
        lastTime = vex::timer::system() - 1; // 前回の時間を更新
      }
      /// @brief 自己位置推定手法初期化
      /// @param pose ロボットの姿勢
      void setPose( Pose pose ) {
        this -> pose = pose; 
        setGyro(pose.w);
      }     
      /// @brief 全てのモータを停止
      void stop() {
        FR.stop(); 
        FL.stop();
        RR.stop();
        RL.stop();
      }
      /// @brief コントローラ操作を行う関数
      /// @param y 望むロボットのy軸出力（−１から１）
      /// @param w 望むロボットの回転出力（−１から１　時計回り）
      void arcadeDrive(float y, float w) {
        w *= W_SCALER; // 定数スカラーを回転出力に掛ける
        float right = y - w; // 右車輪の出力を導く
        float left = y + w;  // 左車輪の出力を導く
        // 右か左が１を超えている場合両値を比例的に減らす
        float max = fmax( fmax( fabs(right), 1 ), left ); 
        right = right/max; 
        left = left/max; 
        drive(left, right); // 左右独立出力関数に入力
      }
      /// @brief 自己位置推定手法を更新
      void localize() {
        pose.w = getGyro(); // イナーシャルセンサによるロボットの角度を更新
        float time = (vex::timer::system() - lastTime) / 1000; // 前回と今回の時差を秒に直す
        lastTime = vex::timer::system(); // 前回時間を初期化
        // エンコーダーの速度をrpmに名をします
        float left  = encoderLeft.velocity(vex::velocityUnits::dps) * DpsToRps; 
        float right = encoderRight.velocity(vex::velocityUnits::dps) * DpsToRps;
        velocity.y = (right + left) / 2; //　右と左の平均をとり、進んだ距離を近似
        velocity.x = 0;                  //　x軸の動きは非ホロノミック系にはありえない
        //　ロボット視点の速度を一般視点に直すためにロボットの角度の分、速度ベクトルを回転します
        velocity.rotate( pose.w );
        Vector dist {velocity.x * time, velocity.y * time}; //　移動ベクトルは速度掛ける時間
        distanceTraveled += dist.getMagnitude(); // 走った距離足す今回走った距離
        // 今回の移動ベクトルを合計位置推定ベクトルに追加
        pose.x += dist.x;
        pose.y += dist.y;
      }
      /// @brief 経路を実行
      /// @param trajectory 走る経路
      /// @return 実行の捗り (0から1)
      float follow(DifferentialTrajectory trajectory) {
        localize(); // 自己位置推定手法を更新
        float progress = fitToRange( distanceTraveled / trajectory.length, 0, 1 ); // 実行捗りを求める
        if ( progress < 1 ) { // 実行が終了わってない限り
          Waypoint waypoint = trajectory.get(distanceTraveled); // 走った距離を用い経路から次の経由地を特定
          //　スプライン補間の場合、PID制御を用いて目的角度を到達するために適切な出力を導く。
          //　概念的には、現在角度と目的角度の最短差を導き、その差が０に近づけるよに出力量を決める
          float w = trajectory.type == spline ? omegaPID.get( wrap(pose.w, waypoint.heading.w) , 0) : 0;
          arcadeDrive( waypoint.heading.y, w ); // 左右独立出力関数に入力
          return progress; //　実行捗りを毎回返す
        }      
        stop();   // モータを全て停止
        return 1; // 経路が無事実行されたことを再び示す
      }
  };

#endif