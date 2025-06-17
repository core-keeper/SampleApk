package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.annotation.SuppressLint;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Quater extends Quaternion implements PIDValue<Quater> {
    public Quater(float x, float y, float z, float w) {
        super(x, y, z, w);
    }

    public Quater(Quaternion h) {
        this(h.getX(), h.getY(), h.getZ(), h.getW());
    }

    public Quater(float magnitude, Quaternion argument) {
        super(
            magnitude * new Quater(argument).normalize().getX(),
            magnitude * new Quater(argument).normalize().getY(),
            magnitude * new Quater(argument).normalize().getZ(),
            magnitude * new Quater(argument).normalize().getW()
        );
    }

    public Quater(float radius, Point axis, float angle) {
        this(
            radius * (float)Math.sin(angle / 2) * (float)new Vector(axis).normalize().getX(),
            radius * (float)Math.sin(angle / 2) * (float)new Vector(axis).normalize().getY(),
            radius * (float)Math.sin(angle / 2) * (float)new Vector(axis).normalize().getZ(),
            radius * (float)Math.cos(angle / 2)
        );
    }

    public Quater(Point axis, float angle) {
        this(1.0f, axis, angle);
    }

    public Quater(Quaternion sign, float image) {
        this(
            (float)Math.exp(new Quater(sign).normalize().getW()),
            new Point(sign.getX(), sign.getY(), sign.getZ()),
            image * 2
        );
    }

    // --- PIDValue 介面實作開始 ---

    @Override
    public Quater identity() {
        return new Quater(0.0f, 0.0f, 0.0f, 1.0f);
    }

    @Override
    public double magnitude() {
        // 通常四元數的 "magnitude" 是指虛部向量的長度，代表旋轉角度的大小。
        return Math.sqrt(getX() * getX() + getY() * getY() + getZ() * getZ());
    }

    @Override
    public Quater gain(double rate) {
        return power(rate);
    }

    @Override
    public Quater absolute(Quater target) {
        return this.mul(target);
    }

    @Override
    public Quater relative(Quater origin) {
        return this.rdiv(origin);
    }

    // --- PIDValue 介面實作結束 ---

    public Quater mul(float scalar) {
        return new Quater(scalar * getX(), scalar * getY(), scalar * getZ(), scalar * getW());
    }

    public Quater mul(Quaternion h) {
        float w1 = getW();
        float x1 = getX();
        float y1 = getY();
        float z1 = getZ();

        float w2 = h.getW();
        float x2 = h.getX();
        float y2 = h.getY();
        float z2 = h.getZ();

        float w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        float x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        float y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        float z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

        return new Quater(x, y, z, w);
    }

    public Quater conj() {
        return new Quater(-getX(), -getY(), -getZ(), getW());
    }

    public float quad() {
        return getW() * getW() + getX() * getX() + getY() * getY() + getZ() * getZ();
    }

    public float norm() {
        return (float)Math.sqrt((double)quad());
    }

    public Quater normalize() {
        float length = norm();
        return length == 0 ? new Quater(0, 0, 0, 0) : mul(1.0f / length);
    }

    public Quater inv() {
        float q_squared = quad();
        if (q_squared == 0) {
            return new Quater(0, 0, 0, 0);
        }
        return conj().mul(1.0f / q_squared);
    }

    public Quater rdiv(Quaternion h) {
        return mul(new Quater(h).inv());
    }

    public Quater ldiv(Quaternion h) {
        return new Quater(h).inv().mul(this);
    }

    public Quater transform(Quaternion original) {
        return mul(original).mul(inv());
    }

    public Quater transform(Point original) {
        return mul(new Quater((float)original.getX(), (float)original.getY(), (float)original.getZ(), 0.0f).mul(inv()));
    }

    /**
     * 返回四元數的實部 (w 分量)。
     * @return 實部值。
     */
    public float realPart() {
        return getW();
    }

    /**
     * 返回四元數的虛部向量 (x, y, z 分量)。
     * @return 由 x, y, z 組成的 Vector 物件。
     */
    public Vector vectorPart() {
        return new Vector(getX(), getY(), getZ());
    }

    /**
     * 計算四元數的自然對數。
     * log(q) = log(||q||) + (v / ||v||) * arccos(w / ||q||)
     * 其中 q = w + v
     * 返回的四元數實部為 log(||q||)，虛部為旋轉軸乘以角度。
     */
    public Quater log() {
        double q_norm = this.norm(); // 四元數的範數
        double w = this.getW();     // 實部
        Vector v = this.vectorPart(); // 虛部向量 (xi + yj + zk)
        double v_norm = v.norm();   // 虛部向量的範數

        if (q_norm == 0) {
            return new Quater(0, 0, 0, 0);
        }

        if (v_norm == 0) { // 純實數四元數 (w, 0, 0, 0)
            if (w > 0) {
                return new Quater(0, 0, 0, (float)Math.log(w));
            } else {
                return new Quater(0, 0, 0, (float)Math.log(Math.abs(w)));
            }
        }

        double theta = Math.acos(w / q_norm);
        Vector axis = v.normalize();

        return new Quater(
            (float)(theta * axis.getX()),
            (float)(theta * axis.getY()),
            (float)(theta * axis.getZ()),
            (float)Math.log(q_norm)
        );
    }

    /**
     * 計算四元數的指數。
     * exp(q) = exp(w + v) = e^w * (cos(||v||) + (v / ||v||) * sin(||v||))
     */
    public Quater exp() {
        double x = getX();
        double y = getY();
        double z = getZ();
        double w = getW(); // 實部

        double exp_w = Math.exp(w);

        Vector v = new Vector(x, y, z);
        double v_norm = v.norm();

        if (v_norm == 0) {
            return new Quater(0, 0, 0, (float)exp_w);
        }

        double sin_v_norm = Math.sin(v_norm);
        double cos_v_norm = Math.cos(v_norm);

        return new Quater(
            (float)(exp_w * (x / v_norm) * sin_v_norm),
            (float)(exp_w * (y / v_norm) * sin_v_norm),
            (float)(exp_w * (z / v_norm) * sin_v_norm),
            (float)(exp_w * cos_v_norm)
        );
    }

    /**
     * 計算四元數的實數次方 q^t = exp(t * log(q))。
     * @param exponent 實數指數。
     * @return 計算結果的四元數。
     */
    public Quater power(double exponent) {
        Quater log_q = this.log();
        Quater scaled_log_q = log_q.mul((float)exponent);
        return scaled_log_q.exp();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("Quater[x=%.3f, y=%.3f, z=%.3f, w=%.3f]", getX(), getY(), getZ(), getW());
    }
}