package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Quater extends Quaternion {
    public Quater(float x, float y, float z, float w) {
        super(x, y, z, w);
    }

    public Quater(Quaternion h) {
        this(h.getX(), h.getY(), h.getZ(), h.getW());
    }

    public Quater(float magnitude, Quaternion argument) {
        super(
            magnitude * new Quater(argument).normalize().getX(),
            magnitude *  new Quater(argument).normalize().getY(),
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
        // scale=log(r^2), vector=sgn(u), sign=(scale, vector), image=angle/2, action=(1/2, image)
        this(
                (float)Math.exp(new Quater(sign).normalize().getW()),
                new Point(sign.getX(), sign.getY(), sign.getZ()),
                image * 2
        );
    }

    public Quater scale(float k) {
        return new Quater(k * getX(), k * getY(), k * getZ(), k * getW());
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
        float lenth = norm();
        return lenth == 0 ? new Quater(0, 0, 0, 0) : scale(1.0f / lenth);
    }

    public Quater inv() {
        return conj().scale(quad());
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
}
