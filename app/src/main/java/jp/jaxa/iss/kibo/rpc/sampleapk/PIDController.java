package jp.jaxa.iss.kibo.rpc.sampleapk;

/**
 * 泛型 PID 控制器，可處理任何實現 PIDValue 介面的型別。
 * @param <T> 必須是實現 PIDValue 介面的型別。
 */
public class PIDController<T extends PIDValue<T>> {
    private double Kp; // 比例增益
    private double Ki; // 積分增益
    private double Kd; // 微分增益

    private T setpoint; // 目標值 (泛型型別)

    private T lastError; // 上一次的誤差 (泛型型別)
    private T integralSum; // 誤差的累積總和 (泛型型別)

    private double integralLimit; // 積分項的標量大小限制 (例如，其 magnitude 的最大值)

    private long lastUpdateTime; // 上次更新時間

    // 建構子
    public PIDController(double Kp, double Ki, double Kd, double integralLimit) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.integralLimit = integralLimit;
        this.lastUpdateTime = System.nanoTime();
    }

    // 可以提供一個不帶 integralLimit 的建構子，使用預設值
    public PIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, Double.MAX_VALUE); // 預設為不限制積分
    }

    // 設定目標值
    public void setSetpoint(T setpoint) {
        this.setpoint = setpoint;
        reset(); // 當設定新的目標值時，重置 PID 狀態
    }

    // 重置控制器狀態
    public void reset() {
        // 使用 identity() 來初始化 lastError 和 integralSum
        if (setpoint != null) {
            this.lastError = setpoint.identity();
            this.integralSum = setpoint.identity();
        } else {
            // 如果 setpoint 尚未設定，無法取得 T 的 identity 實例
            // 在這種情況下，在第一次 update 時檢查並初始化
            this.lastError = null;
            this.integralSum = null;
        }
        this.lastUpdateTime = System.nanoTime();
    }

    /**
     * 更新 PID 計算並返回控制輸出。
     *
     * @param processVariable 當前實際值。
     * @return 計算出的控制輸出（泛型型別 T），代表需要施加的調整量。
     */
    public T update(T processVariable) {
        long now = System.nanoTime();
        double deltaTime = (now - lastUpdateTime) / 1_000_000_000.0; // 將納秒轉換為秒
        lastUpdateTime = now;

        if (setpoint == null) {
            throw new IllegalStateException("Setpoint not set for PID controller.");
        }

        // 1. 計算誤差 (Error = Setpoint - ProcessVariable)
        T error = setpoint.relative(processVariable);

        // 初始化 lastError 和 integralSum (如果尚未初始化)
        if (lastError == null) {
            lastError = error.identity();
        }
        if (integralSum == null) {
            integralSum = error.identity();
        }

        // 2. 計算比例項 (P)
        T proportional = error.gain(Kp);

        // 3. 計算積分項 (I)
        integralSum = integralSum.absolute(error.gain(deltaTime));

        // 積分飽和限制 (Integral Windup Prevention)
        // 如果積分項的 magnitude 超出限制，則將其縮放回限制範圍內
        double currentIntegralMagnitude = integralSum.magnitude();
        if (currentIntegralMagnitude > integralLimit) {
            // 避免除以零，或者當 magnitude 極小時進行不必要的縮放
            if (currentIntegralMagnitude > 1e-9) { // 小於這個值則視為接近零
                double scaleFactor = integralLimit / currentIntegralMagnitude;
                integralSum = integralSum.gain(scaleFactor);
            } else {
                integralSum = integralSum.identity(); // 如果非常接近零，直接設為 identity
            }
        }
        T integral = integralSum.gain(Ki);

        // 4. 計算微分項 (D)
        T derivativeError = error.relative(lastError); // 計算誤差的變化量
        // 處理 deltaTime 過小的情況，避免除以零或數值不穩定
        double safeDeltaTime = (deltaTime == 0) ? 1e-9 : deltaTime; // 使用一個非常小的值代替零
        T derivative = derivativeError.gain(Kd / safeDeltaTime);

        // 5. 計算總輸出
        T output = proportional.absolute(integral).absolute(derivative);

        // 6. 更新 lastError
        lastError = error;

        // 輸出限制：這裡不處理泛型輸出的最終限制。
        // 實際的控制量限制應在 PID 控制器外部，將 T 轉換為實際的物理指令時進行。
        return output;
    }
}