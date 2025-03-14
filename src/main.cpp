#include "lib/Include.h"
#include "lib/HolonomicDrive.h"
#include "lib/Trajectory.h"

using namespace vex;

competition Competition;

//　経路計画を呼び出す
HolonomicTrajectory traj {
  PathPlus {                // 区分的エルミート補間式を定義
    Vector{0,-57},          // 現在地点の定義
    Vector{32.3,22.2},      // 途中地点の定義
    Vector{-30,52},         // 目的地点の定義
    Vector{-95,2},          // 現在角度の定義
    Vector{172.7,101.8},    // 途中角度の定義
    Vector{-65,-1}          // 目的角度の定義
  },
  // 速度プロフィールの定義
  StaticProfile{0.15, 0.05, 0.45, 0.35, 0.8},
  std::vector<HolonomicPose> { // ホロノミック姿勢の配列を定義
    HolonomicPose {0, 0},      // 点A（現在地）では0度を向いている
    HolonomicPose {0.3, 180},  // 点Aと点Bを結ぶ経路が30%終了した時、180度を向いている
    HolonomicPose {1, 300},    // 点B（途中地）では300度を向いている
    HolonomicPose {0.5, 90},   // 点Bと点Cを結ぶ経路が半分終了した時、90度を向いている
    HolonomicPose {2, 5}       // 点C（目的地）では5度を向いている
  }  
};

// ホロノミック車台を宣言
HolonomicDrive drive;

/// @brief プログラムが実行されると最初に呼ばれる関数
/// 通常、センサーやモータの初期化を行う場所
void pre_auton(void) {
  drive.init();  // 車台の初期化関数
  // 2秒処理を停止することで初期化の完了を待つ
  wait(2000, msec);
  // 車台の現在地を自己位置推定手法の原点として入力
  drive.setPose(traj.initialPose);  
}

/// @brief 自動操作の期間に呼ばれる関数
void autonomous(void) {
  // 経路実行が完了しない場合
  while (drive.follow(traj) != 1) {
    drive.localize(); // 自己位置推定手法
    wait(10, msec);   // 処理が詰まらないようループごとに時間を空ける
  }
}

/// @brief 手動操作の期間に呼ばれる関数
void usercontrol(void) {
  while (true) {
    /* 自己位置推定 */
    drive.localize();
    /* コントローラ入力処理 */
    double x = quadradic( master.Axis4.value() );
    double y = quadradic( master.Axis3.value() ); 
    double omega = quadradic( master.Axis1.value());
    drive.arcadeDrive( Vector(x, y), omega ); 
    /* 処理が詰まらないようにループごとに 20msec 空ける */
    wait(20, msec);
  }
}

/// @brief プログラム実行時に最初に呼ばれる関数
int main() {
  Competition.autonomous(autonomous);     // 呼び出し先を設定
  Competition.drivercontrol(usercontrol); // 呼び出し先を設定
  pre_auton();                            // 呼び出し先を設定
  while (true) {
    wait(100, msec);
  }
}
