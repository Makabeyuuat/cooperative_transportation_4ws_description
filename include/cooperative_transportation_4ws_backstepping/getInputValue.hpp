#ifndef GET_INPUT_VALUE_HPP
#define GET_INPUT_VALUE_HPP

#include <vector>
#include <cmath>

class getInputValue {
public:
    // 各状態の微分方程式右辺関数のポインタ型
    using FunctionPtr = double(*)(const std::vector<double>&);

    // /**
    //  * @brief コンストラクタ
    //  * @param h_ 時間刻み（ステップサイズ）
    //  * @param f_ 各状態に対応する微分関数ポインタの配列（サイズは DIM+1）
    //  */
    getInputValue(double h);

    // /**
    //  * @brief ルンゲ‐クッタ法による状態更新と制御入力計算を実行する
    //  * @param x_old 現在の状態（サイズは DIM+1）
    //  * @param sr_j  制御入力計算用の参照インデックス（例：センサ情報など）
    //  */
    void rungeKutta(std::vector<double>& x_old, int sr_j);
    void getU(std::vector<double>& x_old, int sr_j);
    void getXInput(std::vector<double>& x_old, std::vector<double>& x_input);
    // 計算結果の取得用
    const std::vector<double>& getX_new() const;
    double getU4()  const;
    double getU5()  const;
    double getU6()  const;
    double getU7()  const;
    double getU8()  const;
    double getU9()  const;
    double getU10() const;
    double getU11() const;
    double getU12() const;

private:
    double h;  // 時間刻み

    // ルンゲ‐クッタ法用中間計算変数（サイズはそれぞれ DIM+1）
    std::vector<std::vector<double>> k; // k[][0..3]
    std::vector<std::vector<double>> r; // r[][0..3]
    std::vector<std::vector<double>> q; // q[][0..3]
    std::vector<std::vector<double>> x; // x[0], x[1], x[2]

    // differential_equations.hpp で定義された配列をコピー
    std::vector<FunctionPtr> fAllVec;

    // ここから内部で制御入力を計算する関数群

    void U1(const std::vector<double>& x_old, int sr_j);
    void U2(const std::vector<double>& x_old, int sr_j);
    void U3(const std::vector<double>& x_old, int sr_j);
    void U4_U5_U6(const std::vector<double>& x_old, int sr_j);
    void U7_U8_U9(const std::vector<double>& x_old, int sr_j);
    void U10_U11_U12(const std::vector<double>& x_old, int sr_j);
};

#endif // GET_INPUT_VALUE_HPP
