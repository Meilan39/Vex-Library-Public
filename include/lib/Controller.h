#ifndef CONTROLLER 
#define CONTROLLER 

  #include "lib/Include.h"
  #include "lib/Vector.h"

    /// @brief コントローラーの画面を消す
    void clear() {
        master.Screen.clearScreen();
    }
    /// @brief コントローラーに真偽値を出力する
    /// @param row 出力する行（1から5）
    /// @param T 出力する真偽値
    void print( int row, bool T ) {
        master.Screen.clearLine(row);
        master.Screen.setCursor(row, 2);
        master.Screen.print(T);
    }
    /// @brief コントローラーに float を出力
    /// @param row 出力する行（1から5）
    /// @param T 出力する float
    void print( int row, float T ) {
        master.Screen.clearLine(row);
        master.Screen.setCursor(row, 2);
        master.Screen.print(T);
    }
    /// @brief コントローラーに double を出力する
    /// @param row 出力する行（1から5）
    /// @param T 出力する double
    void print( int row, double T ) {
        master.Screen.clearLine(row);
        master.Screen.setCursor(row, 2);
        master.Screen.print(T);
    }
    /// @brief コントローラーに int を出力する
    /// @param row 出力する行（1から5）
    /// @param T 出力する int
    void print( int row, int T ) {
        master.Screen.clearLine(row);
        master.Screen.setCursor(row, 2);
        master.Screen.print(T); 
    }
    /// @brief コントローラーに文字列を出力する
    /// @param row 出力する行（1から5）
    /// @param T 出力する文字列
    void print( int row, char T[] ) {
        master.Screen.clearLine(row);
        master.Screen.setCursor(row, 2);
        master.Screen.print(T);
    }
    /// @brief コントローラーのジョイスティックを -1 から 1 に正規化する
    /// @param raw 軸の値の参照
    void normalize( float &raw ) {
        raw = raw / 127;
    }
    /// @brief コントローラーのジョイスティックにデッドバンドを適応
    /// @param raw 軸の値の参照
    void banded( float &raw ) {
        raw = fabs(raw) > BAND ? raw : 0;
    }
    /// @brief 正方形のコントローラー軸を単位円にマップする
    /// @param x ジョイスティックの x 軸
    /// @param y ジョイスティックの y 軸
    Vector map( Vector coordinate ) {
        coordinate.scale( sqrt( 1 - coordinate.y * coordinate.y / 2 ) );
        return coordinate;
    }
    /// @brief ジョイスティックの標準処理
    /// @param raw ジョイスティックの値
    /// @return 正規化されたデッドバンド付与の
    float normal( float raw ) {
        banded(raw);
        normalize(raw);
        return raw;
    }
    /// @brief ジョイスティックの二次関数スムージング処理
    /// @param raw ジョイスティックの値
    /// @return 標準処理したジョイスティックの値に二次関数スムージングを施したもの
    float quadradic( float raw ) {
        banded(raw);
        normalize(raw);
        return raw>0 ? raw*raw : -raw*raw;
    }
    /// @brief ジョイスティくの三次関数スムージング処理
    /// @param raw ジョイスティックの値
    /// @return 正規化したジョイスティックの値に三次関数スムージングを施したもの
    float cubic( float raw ) {
        normalize(raw);
        return raw*raw*raw;
    }

    /// @brief トグルスイッチのクラス
    class Toggle {
        bool value = false;
        bool lastValue = false;
      public:
        bool get( bool raw ) {
            if (raw!=lastValue && raw) value = !value;
            lastValue = raw;
            return value;
        }
    };

#endif