#ifndef HOLONOMICDRIVE
#define HOLONOMICDRIVE

  #include "lib/Include.h"
  #include "lib/Vector.h"
  #include "lib/Pose.h"
  #include "lib/Trajectory.h"
  #include "lib/PID.h"
  #include "lib/Helpers.h"

  #include "lib/Controller.h"

  // 一般的ホロノミック系ロボット車台（x-drive)
  class HolonomicDrive {
    private:
        vex::motor FR {FR_id, vex::gearSetting::ratio18_1, true }; // 右前のモータを定義
        vex::motor FL {FL_id, vex::gearSetting::ratio18_1, false}; // 左前のモータを定義
        vex::motor RL {RL_id, vex::gearSetting::ratio18_1, false}; // 左後ろのモータを定義
        vex::motor RR {RR_id, vex::gearSetting::ratio18_1, true }; // 右後ろのモータを定義
        vex::rotation encoderRight {encoderRight_id}; // 右の車輪に付いているエンコーダー
        vex::rotation encoderLeft {encoderLeft_id};   // 左の車輪に付いているエンコーダー
        vex::rotation encoderRear {encoderRear_id};   // 後ろの車輪に付いているエンコーダー
        vex::inertial inertial {inertial_id, vex::turnType::left}; // イナーシャルセンサの定義
    public:
        Pose pose = {0,0,0};      // ロボットの姿勢オブジェクトを宣言
        Vector velocity = {0,0};  // ロボットの速度オブジェクトを宣言
        int progress = 0;         // 経路実行の捗り
        bool fieldCentric = true; // 運転士視点操作
    private:
        const float ODOMETRY_WHEEL_DIAMETER = 2.75; // 車輪の直径
        const float WHEEL_MAX_RPM = 180; // 最高速度の定数（rpm）
        const float DpsToRps = ODOMETRY_WHEEL_DIAMETER * PI / 360; // dps（度毎秒）と　ips（インチ毎分）の変更用
        Vector FR_component {135}; // 北西に向く単位ベクトルは右前モータの方向進行
        Vector FL_component {45};  // 北東に向く単位ベクトルは左前モータの方向進行
        Vector RL_component {135}; // 北西に向く単位ベクトルは左後ろモータの方向進行
        Vector RR_component {45};  // 北東に向く単位ベクトルは右後ろモータの方向進行
        PID omegaPID {0.015, 0, 0, 0.008, -1, 1}; // PID制御クラスの定義
    private:
        float lastTime = 0;         // 前ループ記録した時間
        float distanceTraveled = 0; // 走った距離
    private:
        /// @brief イナーシャルセンサの角度を変更
        /// @param angle 角度（度数）
        void setGyro( float angle ) {
            inertial.setHeading(angle, vex::rotationUnits::deg);
        }
        /// @brief ロボットの角度をイナーシャルセンサに問う
        /// @return ロボットの角度（度数）
        float getGyro() {
            return inertial.heading();
        }
    public:
        /// @brief 車台の初期化
        void init() {
            inertial.calibrate(); // イナーシャルセンサの初期化
            FR.setBrake(vex::brakeType::brake); // 右前のモータをブレークモードに設定
            FL.setBrake(vex::brakeType::brake); // 左前のモータをブレークモードに設定
            RL.setBrake(vex::brakeType::brake); // 左後ろのモータをブレークモードに設定
            RR.setBrake(vex::brakeType::brake); // 右後ろのモータをブレークモードに設定
            encoderLeft.setReversed(true);   // 左エンコーダーの方向を設定
            encoderRight.setReversed(false); // 右エンコーダーの方向を設定
            encoderRear.setReversed(false);  // 後ろエンコーダーの方向を設定
            while (inertial.isCalibrating()) wait(20, msec); // センサの初期化処理を待つ
        }
        /// @brief 自己位置推定手法初期化
        /// @param pose ロボットの姿勢
        void setPose( Pose pose ) {
            this -> pose = pose;
            setGyro( pose.w );      
        }
        /// @brief 自己位置推定手法を更新
        void localize() {
            pose.w = getGyro(); // イナーシャルセンサによるロボットの角度を更新
            float time = (vex::timer::system() - lastTime) / 1000; // 前回と今回の時差を秒に直す
            lastTime = vex::timer::system(); // 前回時間を初期化
            // エンコーダーの速度をrpmに名をします
            float left  = encoderLeft.velocity(vex::velocityUnits::dps) * DpsToRps;
            float right = encoderRight.velocity(vex::velocityUnits::dps) * DpsToRps;
            float rear  = encoderRear.velocity(vex::velocityUnits::dps) * DpsToRps;
            float rot = (left - right) / 2; // 回転速度は右と左の差を２で割ったもの
            velocity.x = rear - rot; //　x軸の速度は後ろの速度から回転速度を引いたもの
            velocity.y = ( (right - rot) + (left + rot) ) / 2; // y軸の速度は回転を補った右と左の平均
            // ロボット視点の速度を一般視点に直すためにロボットの角度の分、速度ベクトルを回転します
            velocity.rotate( pose.w );
            Vector dist {velocity.x * time, velocity.y * time}; //　移動ベクトルは速度掛ける時間
            distanceTraveled += dist.getMagnitude(); // 走った距離足す今回走った距離
            // 今回の移動ベクトルを合計位置推定ベクトルに追加
            pose.x += dist.x; 
            pose.y += dist.y;
        }
        /// @brief コントローラ操作を行う関数
        /// @param translation 望む平面横断を表す単位ベクトル
        /// @param w 望む回転速度（ー１から１）
        void arcadeDrive( Vector translation, float w ) {
            // 運転士視点操作の場合得られた横断ベクトルをロボットの角度の分、逆回転させます
            if (fieldCentric) translation.rotate( -pose.w );
            float fr = ( translation.y * FR_component.y) + ( translation.x * FR_component.x) - w;
            float fl = ( translation.y * FL_component.y) + ( translation.x * FL_component.x) + w;
            float rl = ( translation.y * RL_component.y) + ( translation.x * RL_component.x) + w;
            float rr = ( translation.y * RR_component.y) + ( translation.x * RR_component.x) - w;
            float max = fmax( fmax( fabs(fr), fabs(fl) ), fmax( fabs(rl), fabs(rr) ) ); // 一番高い値を探す
            if (max > 1) { // その値が１より高ければ全ての値を比例的に減らす（１以下に制限）
                fr /= max; // 一番でかい値で割る（１以下になる）
                fl /= max; // 一番でかい値で割る（１以下になる）
                rl /= max; // 一番でかい値で割る（１以下になる）
                rr /= max; // 一番でかい値で割る（１以下になる）
            }
            fr *= WHEEL_MAX_RPM;  // 適当な速度を導く
            fl *= WHEEL_MAX_RPM;  // 適当な速度を導く
            rl *= WHEEL_MAX_RPM;  // 適当な速度を導く
            rr *= WHEEL_MAX_RPM;  // 適当な速度を導く
            FR.spin(forward, fr, vex::velocityUnits::rpm);  // モータに速度命令
            FL.spin(forward, fl, vex::velocityUnits::rpm);  // モータに速度命令
            RL.spin(forward, rl, vex::velocityUnits::rpm);  // モータに速度命令
            RR.spin(forward, rr, vex::velocityUnits::rpm);  // モータに速度命令
        }
        /// @brief 全てのモータを停止
        void stop() {
            FR.stop();
            FL.stop();
            RL.stop();
            RR.stop();
        }
        /// @brief 経路を実行
        /// @param trajectory 走る経路
        /// @return 実行の捗り (0から1)
        float follow(HolonomicTrajectory trajectory) {
            localize(); // 自己位置推定手法を更新
            float progress = fitToRange( distanceTraveled / trajectory.length, 0, 1 ); // 実行捗りを求める
            if ( progress < 1 ) { // 実行が終了わってない限り
                Waypoint waypoint = trajectory.get(distanceTraveled); // 走った距離を用い経路から次の経由地を特定
                //　ホロノミック姿勢の場合、PID制御を用いて目的角度を到達するために適切な出力を導く。
                //　概念的には、現在角度と目的角度の最短差を導き、その差が０に近づけるよに出力量を決める
                float w = trajectory.orientation ? omegaPID.get( wrap(pose.w, waypoint.heading.w) , 0) : 0;
                arcadeDrive( Vector {waypoint.heading.x, waypoint.heading.y}, w ); // コントローラ操作の関数に入力
                return progress; //　実行捗りを毎回返す
            }      
            stop();   // モータを全て停止
            return 1; // 経路が無事実行されたことを再び示す
        }
  };

#endif