#ifndef TRAJECTORY 
#define TRAJECTORY

  #include "lib/Include.h"
  #include "lib/Helpers.h"
  #include "lib/Vector.h"
  #include "lib/Pose.h"
  #include "lib/VelocityProfile.h"

  /// @brief　補間方法を選択できる列挙型
  /// @param linear 直線補間
  /// @param spline スプライン補間
  enum PathType { linear, spline };

  /// @brief エルミート補間式を定義できる構造体
  /// @param p0 現在地点
  /// @param p1 目的地点
  /// @param t0 現在角度
  /// @param t1 目的角度
  struct Path {
    Vector p0;
    Vector p1;
    Vector t0;
    Vector t1;
  };

  /// @brief 区分的エルミート補間式を定義する構造体
  /// @param p0　現在地点
  /// @param p1　途中地点
  /// @param p2　目的地点
  /// @param t0　現在角度
  /// @param t1　途中角度
  /// @param t2　目的角度
  struct PathPlus {
    Vector p0;
    Vector p1;
    Vector p2;
    Vector t0;
    Vector t1;
    Vector t2;
  };

  /// @brief 生成された経路がこの構造体の配列で表されている
  /// @param type 補間方法（直線かスプライン）
  /// @param dist　経路の始点からの距離
  /// @param heading　ロボットの姿勢
  struct Waypoint {
    PathType type;
    float dist;
    Pose heading;
  };

  /// @brief 目的のホロノミック姿勢を経路の特定の処理位置に登録（ホロノミック姿勢はホロノミック車台の角度を示します。
  /// ホロノミック系のロボットは平面的横断と回転を同時に行う機能を持ち、進行方向と別の角度を保つことができる）。
  /// @param dist 特定する処理位置 (0 から 1)
  /// @param angle　ロボットの角度
  struct HolonomicPose {
    float dist;
    float angle;
  };

  /// @brief エルミート補間式にある処理位置（x）を問い、そ地点の姿勢を返す
  /// @param path エルミート補間式の定義
  /// @param previous 前回の処理位置の姿勢
  /// @param x 処理位置（0から１）
  /// @return 処理位置（x）のロボット姿勢
  Pose CubicHermiteInterpolation(Path path, Pose previous, float x) {
    // エルミート補間多項式の表現
    float h1 = 2*(x*x*x) - 3*(x*x) + 1; //　１から始まり０に向かって低下する
    float h2 = -2*(x*x*x) + 3*(x*x);    //　０から始まり１に向かって上昇する
    float h3 = (x*x*x) - 2*(x*x) + x;   //　序盤に上に膨らみ終盤に低下する
    float h4 = (x*x*x) - (x*x);         //　終盤に下に膨らみ序盤に低下する
    // エルミート補間定義に従い処理位置（x）の姿勢を導く
    Pose current; //　姿勢オブジェクトを作成
    current.x = path.p0.x * h1 + path.p1.x * h2 + path.t0.x * h3 + path.t1.x * h4;  //　x　値を導く
    current.y = path.p0.y * h1 + path.p1.y * h2 + path.t0.y * h3 + path.t1.y * h4;  //　y　値を導く
    //　今回の位置から前回の位置を引くことでその差を表すベクトルを生成
    //　生成されたベクトルの角度を導き経路の角度を近似することができる
    current.w = Vector {current.x - previous.x, current.y - previous.y}.getAngle(); 
    return current; //　姿勢を返す
  }

  /// @brief ホロノミック姿勢の std::vector をある処理位置　x で補間。
  /// 目的姿勢を提示された経路位置に厳密に達成する為に滑らか且つ徐々に近づいていく必要がありました。
  /// この関数は提示された処理位置を用いて全ての処理位置のあるべき姿勢を導く役割を果たしています。
  /// @param orientation ホロノミック姿勢の　std::vector （処理位置０と１の姿勢は必ず定義されている）
  /// @param x 処理位置
  /// @return 補間値
  float InterpolateHolonomicPose(std::vector<HolonomicPose> orientation, float x ) {
    if ( ! orientation.empty() ) { //　ホロノミック姿勢が示されているか
      int s = 0; //　イテレータ初期化
      //　std::vector から現在の処理位置（x）が入る区間を探る
      //　区間の先頭と後尾の角度と処理位置の差を取ることで直線補間を行うことができる
      while (orientation.at(s).dist < x) { s++; if (s == orientation.size() - 1) break; } //　std::vector を探
      float angleError = wrap(orientation.at(s-1).angle, orientation.at(s).angle);        //　区間の最短角度差を求める
      float distError = orientation.at(s).dist - orientation.at(s-1).dist;                //　区間の処理位置の差を求める
      x -= orientation.at(s-1).dist;                                                      //  
      return bound( (x / distError) * angleError + orientation.at(s-1).angle );           //  直線補間を行う
    } else {
      return -1; //　ホロノミック姿勢が示されてない場合（ー１）を返す
    }
  }

  /// @brief 非ホロノミック系ロボットの経路計画クラス
  class DifferentialTrajectory {
    public:
      std::vector<Waypoint> waypoints; //　最終的の軌道を表す　std::vector 
      Pose initialPose {0,0,0}; //　初期姿勢
      Pose finalPose {0,0,0};   //　最終姿勢
      PathType type;     //　補間方法
      float length = 0;  //　補間式の長さ
      int index = 0;     //　イテレータ
      bool reverse;      //　経路を逆走行したいか
    public:
      /// @brief 直線補間軌道を生成するコンストラクター
      /// @param trajectory1D 動きたい距離（単位はインチ）(負の値も適用)
      /// @param profile 速度プロフィール
      DifferentialTrajectory(float trajectory1D, StaticProfile profile) {;
          // 100個の経由地を生成しそれぞれの距離と角度を求めます
          for(int i = 1; i <= 100; i++) { //　100回繰り返される（イテレータは1から）
            float x = 0.01 * i;  //　0から1の処理位置を演算
            Waypoint waypoint;   //　経由地を作成
            waypoint.dist = x * fabs(trajectory1D); //　処理位置に基づき距離を導く
            waypoint.heading.y = copysign(profile.get(i), trajectory1D); //　処理位置に基づき走るべき速度を導く
            waypoints.push_back( waypoint ); //　軌道に経由地を追加
          }
          this -> type = linear;                //　補間方法代入
          this -> length = fabs(trajectory1D);  //  補間式の長さを代入
      }
      /// @brief スプライン補間式を生成するコンストラクター（点Aと点Bのみで表せる経路に使用）
      /// @param path エルミート補間式の定義
      /// @param profile 速度プロフィール
      /// @param reverse OPTIONAL: 経路を逆走走したいか
      DifferentialTrajectory(Path path, StaticProfile profile, bool reverse = false) {
          //（generate）関数を呼び点Aから点Bの間の補間を行う
          this -> waypoints = generate(path, reverse, 100, profile); 
          this -> reverse = reverse; //  逆走ブール代入
          this -> type = spline;     //　補間方法代入
      }
      /// @brief 区分的スプライン補間式を生成するコンストラクター（点A、点B、点C、で表す経路に使用）
      /// これ以上の制御性を必する経路は分割すべきだと考えられます
      /// @param path 区分的エルミート補間式の定義
      /// @param profile 速度プロフィール
      /// @param reverse OPTIONAL: 経路を逆走行したいか
      DifferentialTrajectory(PathPlus path, StaticProfile profile, bool reverse = false) {
          //（generate）関数を呼び点Aから点Bの間の補間を行う（明瞭度を100の半分に設定）
          this -> waypoints = generate( Path {path.p0, path.p1, path.t0, path.t1}, reverse, 50, profile ); 
          Pose tempInitialPose = initialPose; //　この時点で初期姿勢は点A。この姿勢を保存します
          float tempLength = length;          //　この時点で経路の長さは点Aから点Bの補間式の長さ。この長さを保存します
          //（generate）関数を呼び点Bから点Cの間の補間を行う（明瞭度を100の半分に設定）
          std::vector<Waypoint> waypoints2 = generate( Path {path.p1, path.p2, path.t1, path.t2}, reverse, 50, profile );
          this -> initialPose = tempInitialPose; //　事前に保存した点Aの姿勢を真の初期姿勢に代入
          this -> length = tempLength + length;  //　点Aから点Bの長さを点Bから点Cの長さに足し真の長さに代入
          //　点Aから点Bの軌道を点Bから点Cの軌道と合体
          this -> waypoints.insert(waypoints.end(), waypoints2.begin(), waypoints2.end()); 
          this -> reverse = reverse; //　逆走ブール代入
          this -> type = spline;     //　補間方法代入
      }
      /// @brief 軌道を生成する関数
      /// @param path エルミート補間式の定義
      /// @param reverse 経路を逆走したいか
      /// @param clarity 明瞭度を示す（一つの経路は100と定められている）
      /// @param profile 速度プロフィール
      /// @return 生成された軌道
      std::vector<Waypoint> generate(Path path, bool reverse, int clarity, StaticProfile profile) {
          float segment = 1.0 / clarity; //　処理位置の一つ一つの区間の長さを導く
          float dist = 0; //　経路の長さを初期化
          // 現在姿勢と前回姿勢を宣言
          Pose previous {path.p0.x, path.p0.y, path.t0.getAngle()}; //　点Aの姿勢に設定　
          Pose current {0, 0, 0}; //　初期化
          //　軌道となる経由地の配列を作成
          std::vector<Waypoint> waypoints;
          //　明瞭度の分繰り返される（イテレータは1から始める）
          for (int i = 1; i <= clarity; i++) {
              // 現在処理位置を求める
              float x = segment * i;
              // 処理位置を元に現在の姿勢を求める
              current = CubicHermiteInterpolation(path, previous, x);
              // 現在と前回の姿勢の差を（previous）に導入
              previous = previous.getError(current);
              // 現在角度と前回角度の差を比例拡大して逆数を取ります（この値は経路の曲率が高いほど小さくなります）
              // 速度プロフィールの現在処理値値を計算（区分的補間の場合、二番目の補間の際　index　が50となっている）
              // 上記の値はどちらとも0から1の範囲で、掛け合わせることで現在処理位置での速度を導けます。
              float speed = (1 / (autonomous_rotation_scaler * fabs(previous.w) + 1)) * (profile.get(i + index));
              // 経由地に代入していきます
              Waypoint waypoint;
              waypoint.dist = length + dist + previous.getVector().getMagnitude(); //　各経由地間の距離の合計
              waypoint.heading.x = 0; //　非ホロノミック系ロボットは横行できません
              waypoint.heading.y = reverse ? -speed : speed; //　以前計算した速度の符号を逆走ブールによって決める
              // ベクトルの差で計算した角度は０が右にありますがロボットのジャイロスコープは０が上にあるため90度を引きます。
              // 逆走の場合ロボットは反対の角度に向く必要があるので180度を足します。最後に角度を０から360度に制限する関数に通します。
              waypoint.heading.w = bound(current.w - 90 + (reverse ? 180 : 0));
              waypoints.push_back(waypoint);// 経由地を軌道に加えます
              // 次のループに備える
              dist = dist + previous.getVector().getMagnitude(); // 今回の経由地間を合計距離に足す
              previous = current; // 今回の姿勢を前回の姿勢に代入
          }
          // 前と同じ理由で初期姿勢と最終姿勢に90度を引き、逆走の場合180度を足します。
          this -> initialPose = Pose {path.p0.x, path.p0.y, bound( path.t0.getAngle() - 90 + (reverse ? 180 : 0) )};
          this ->   finalPose = Pose {path.p1.x, path.p1.y, bound( path.t1.getAngle() - 90 + (reverse ? 180 : 0) )};
          this ->      length = dist;    // 経路の最終的長さは経由地間の距離の合計となります
          this ->       index = clarity; // 区分的補間を行う場合速度プロフィールを継げる為
          return waypoints; // 軌道を呼び出し主に返す
      }
      /// @brief ある距離の入力に対し実行すべき経由地が返される
      /// @param distanceTraveled ロボットが進んだ距離（単位はインチ）
      /// @return 経由地
      Waypoint get(float distanceTraveled) {
          int i = 0;
          while (this->waypoints.at(i).dist < distanceTraveled) i++; // 軌道を探りちょうど次の経由地を特定
          return waypoints.at(i); // 経由地を返す
      }
  };

  /// @brief ホロノミック系ロボットの経路計画クラス
  class HolonomicTrajectory {
    public:
      std::vector<Waypoint> waypoints; //　最終的の軌道を表す　std::vector 
      Pose initialPose {0,0,0}; //　初期姿勢
      Pose finalPose {0,0,0};   //　最終姿勢
      PathType type;     //　補間方法
      bool orientation;  //　ホロノミック姿勢が示されているか
      int index = 0;     //　イテレータ
      int aIndex = 0;    //　ホロノミック姿勢イテレータ（区分的補間の際に使用）
      float length = 0;  //　補間式の長さ
    public:
      /// @brief 直線補間軌道を生成するコンストラクター
      /// @param trajectory2D 目的移動を示すベクトル（単位はインチ）
      /// @param orientation OPTIONAL:  ホロノミック姿勢の　std::vector （処理位置０と１の姿勢は必ず定義されている）
      /// @param profile 速度プロフィール
      HolonomicTrajectory(Vector trajectory2D, StaticProfile profile, std::vector<HolonomicPose> orientation = {}) {
          float angle = trajectory2D.getAngle() / RadToDeg; // 移動ベクトルの角度（度数）を保存
          float distance = trajectory2D.getMagnitude();     // 移動ベクトルの長さ（インチ）を保存
          // 100個の経由地を生成しそれぞれの距離と角度を求めます
          for(int i = 1; i <= 100; i++) { //　100回繰り返される（イテレータは1から）
            float x = 0.01 * i;  //　0から1の処理位置を演算
            Waypoint waypoint;   //　経由地を作成
            float speed = profile.get(i); // 処理位置を速度プロフィールに問い保存
            waypoint.dist = distance * x; // 以前保存した長さから処理位置の距離を図る
            // ロボットを最終的に動かす関数がコントローラの入力を予想している為、アナログスティックの出力の真似をします
            // アナログスティックの出力の模倣は、進行方向と同じ角度の単位ベクトルで、その方向に全速力で進むことを意味する
            // 速度にかけることで適切な速度規制を可能とします
            waypoint.heading.x = speed * cosf(angle); // 移動ベクトルの　x　値に速度を掛ける
            waypoint.heading.y = speed * sinf(angle); // 移動ベクトルの　y　値に速度を掛ける
            // この処理位置で以前定義した「ホロノミック姿勢補間関数」を呼び出しあるべき角度を保存
            waypoint.heading.w = InterpolateHolonomicPose(orientation, x);
            waypoints.push_back( waypoint ); // 軌道に経由地を追加
          }
          this -> type = linear;                      // 補間方法代入
          this -> orientation = !orientation.empty(); // ホロノミック姿勢ブールを代入
          this -> length = distance;                  // 補間式の長さを代入
      }
      /// @brief スプライン補間式を生成するコンストラクター（点Aと点Bのみで表せる経路に使用）
      /// @param path エルミート補間式の定義
      /// @param orientation OPTIONAL: ホロノミック姿勢の　std::vector （範囲は０から１〜処理位置０と１の姿勢は必ず定義）
      /// @param profile 速度プロフィール
      HolonomicTrajectory(Path path, StaticProfile profile, std::vector<HolonomicPose> orientation = {}) {
          //（generate）関数を呼び点Aから点Bの間の補間を行う
          this -> waypoints = generate(path, orientation, 100, profile);
          this -> orientation = !orientation.empty();    //　ホロノミック姿勢ブールを代入
          this -> type = spline;                         //　補間方法代入
      }
      /// @brief 区分的スプライン補間式を生成するコンストラクター（点A、点B、点C、で表す経路に使用）。
      /// これ以上の制御性を必する経路は分割すべきだと考えられます。
      /// @param path 区分的エルミート補間式の定義
      /// @param orientation OPTINAL: ホロノミック姿勢の　std::vector 
      /// 点Aから点Bの範囲は０から１、点Bから点Cの範囲は１から２（処理位置０と２は必ず定義）
      /// @param profile 速度プロフィール
      HolonomicTrajectory(PathPlus path, StaticProfile profile, std::vector<HolonomicPose> orientation = {}) {
          //（generate）関数を呼び点Aから点Bの間の補間を行う（明瞭度を100の半分に設定）
          this -> waypoints = generate( Path {path.p0, path.p1, path.t0, path.t1}, orientation, 50, profile );
          Pose tempInitialPose = initialPose; //　この時点で初期姿勢は点A。この姿勢を保存します
          float tempLength = length;          //　この時点で経路の長さは点Aから点Bの補間式の長さ。この長さを保存します
          //（generate）関数を呼び点Bから点Cの間の補間を行う（明瞭度を100の半分に設定）
          std::vector<Waypoint> waypoints2 = generate( Path {path.p1, path.p2, path.t1, path.t2}, orientation, 50, profile );
          this -> initialPose = tempInitialPose; //　事前に保存した点Aの姿勢を真の初期姿勢に代入
          this -> length = tempLength + length;  //　点Aから点Bの長さを点Bから点Cの長さに足し真の長さに代入
          //　点Aから点Bの軌道を点Bから点Cの軌道と合体
          this -> waypoints.insert(waypoints.end(), waypoints2.begin(), waypoints2.end());
          this -> orientation = !orientation.empty(); // 　ホロノミック姿勢ブールを代入
          this -> type = spline;                      //　補間方法代入
      }
      /// @brief 軌道を生成する関数
      /// @param path エルミート補間式の定義
      /// @param orientation ホロノミック姿勢の　std::vector 
      /// @param clarity 明瞭度を示す（一つの経路は100と定められている）
      /// @param profile 速度プロフィール
      std::vector<Waypoint> generate(Path path, std::vector<HolonomicPose> orientation, int clarity, StaticProfile profile) {
          float segment = 1.0 / clarity; //　処理位置の一つ一つの区間の長さを導く
          float dist = 0; //　経路の長さを初期化
          // 現在姿勢と前回姿勢を宣言
          Pose previous {path.p0.x, path.p0.y, path.t0.getAngle()}; //　点Aの姿勢に設定　
          Pose current {0, 0, 0}; //　初期化
          //　軌道となる経由地の配列を作成
          std::vector<Waypoint> waypoints;
          //　明瞭度の分繰り返される（イテレータは1から始める）
          for (int i = 1; i <= clarity; i++) {
              // 現在処理位置を求める
              float x = segment * i;
              // 処理位置を元に現在の姿勢を求める
              current = CubicHermiteInterpolation(path, previous, x);
              // 現在と前回の姿勢の差を（previous）に導入
              previous = previous.getError(current);
              // 現在角度と前回角度の差を比例拡大して逆数を取ります（この値は経路の曲率が高いほど小さくなります）
              // 速度プロフィールの現在処理値値を計算（区分的補間の場合、二番目の補間の際　index　が50となっている）
              // 上記の値はどちらとも0から1の範囲で、掛け合わせることで現在処理位置での速度を導けます。
              float speed = (1 / (autonomous_rotation_scaler * fabs(previous.w) + 1)) * (profile.get(i + index));
              // 経由地に代入していきます
              Waypoint waypoint;
              waypoint.dist = length + dist + previous.getVector().getMagnitude(); //　各経由地間の距離の合計
              // ロボットを最終的に動かす関数がコントローラの入力を予想している為、アナログスティックの出力の真似をします
              // アナログスティックの出力の模倣は、進行方向と同じ角度の単位ベクトルで、その方向に全速力で進むことを意味する
              // 速度にかけることで適切な速度規制を可能とします
              waypoint.heading.x = cosf(previous.getVector().getAngle() / RadToDeg) * speed; 
              waypoint.heading.y = sinf(previous.getVector().getAngle() / RadToDeg) * speed;
              // この処理位置で以前定義した「ホロノミック姿勢補間関数」を呼び出しあるべき角度を保存                
              waypoint.heading.w = InterpolateHolonomicPose(orientation, aIndex + x);
              waypoints.push_back(waypoint);// 経由地を軌道に加えます
              // 次のループに備える
              dist = dist + previous.getVector().getMagnitude(); // 今回の経由地間を合計距離に足す
              previous = current; // 今回の姿勢を前回の姿勢に代入
          }
          // 初期姿勢と最終姿勢を定義。ホロノミック姿勢が示されていたら従って代入
          this -> initialPose = Pose {path.p0.x, path.p0.y, orientation.empty() ? 0 : orientation.front().angle};
          this ->   finalPose = Pose {path.p1.x, path.p1.y, orientation.empty() ? 0 : orientation.back().angle };
          this ->      length = dist;    // 経路の最終的長さは経由地間の距離の合計となります
          this ->      aIndex = 1;       // 区分的補間を行う場合ホロノミック姿勢をつける為
          this ->       index = clarity; // 区分的補間を行う場合速度プロフィールを継げる為
          return waypoints;              // 軌道を呼び出し主に返す
      }
      /// @brief ある距離の入力に対し実行すべき経由地が返される
      /// @param distanceTraveled ロボットが進んだ距離（単位はインチ）
      /// @return 経由地 
      Waypoint get(float distanceTraveled) {
          int i = 0;
          while (this -> waypoints.at(i).dist < distanceTraveled) i++; // 軌道を探りちょうど次の経由地を特定
          return waypoints.at(i); // 経由地を返す
      }
  };

#endif
