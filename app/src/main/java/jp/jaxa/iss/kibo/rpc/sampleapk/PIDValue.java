package jp.jaxa.iss.kibo.rpc.sampleapk;

/**
 * 定義泛型數值型別所需的操作介面。
 * 任何要被泛型 PID 控制器處理的型別都必須實現此介面。
 * T 是該型別本身。
 */
public interface PIDValue<T extends PIDValue<T>> {
    /**
     * 返回此型別的單位元。
     * 這對於初始化誤差或某些計算很關鍵。
     * PID 的 reset 和某些計算可能需要單位元或是原點值。
     *
     * @return 零值或單位值。
     */
    T identity();

    /**
     * 返回此值的純量大小或範數。
     * 這對於 PID 限制積分項非常有用。
     *
     * @return 值的純量大小。
     */
    double magnitude();

    /**
     * 將當前值按比例縮放。
     * 以李代數來看，概念上等同於：this * rate。
     *
     * @param rate 縮放因子。
     * @return 縮放的值。
     */
    T gain(double rate);

    /**
     * 將當前值與基準值相加。
     * 以李代數來看，概念上等同於：this + target。
     *
     * @param target 基準值。
     * @return 絕對的值。
     */
    T absolute(T target);

    /**
     * 計算當前值相對於基準值的相對值 (誤差)。
     * 以李代數來看，概念上等同於：this - origin。
     *
     * @param origin 基準值。
     * @return 相對的值。
     */
    T relative(T origin);
}