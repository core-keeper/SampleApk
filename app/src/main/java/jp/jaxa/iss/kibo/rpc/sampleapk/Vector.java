package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.annotation.SuppressLint;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Vec3d;

public class Vector extends Point implements PIDValue<Vector> {
    public Vector(double x, double y, double z) {
        super(x, y, z);
    }

    public Vector(Point p) {
        this(p.getX(), p.getY(), p.getZ());
    }

    public Vector(Vec3d p) {
        this(p.toArray()[0], p.toArray()[1], p.toArray()[2]);
    }

    // --- PIDValue 介面實作開始 ---

    @Override
    public Vector identity() {
        return new Vector(0, 0, 0);
    }

    @Override
    public double magnitude() {
        return norm();
    }

    @Override
    public Vector gain(double rate) {
        return mul(rate);
    }

    @Override
    public Vector absolute(Vector target) {
        return add(target);
    }

    @Override
    public Vector relative(Vector origin) {
        return sub(origin);
    }

    // --- PIDValue 介面實作結束 ---

    public Vector mul(double scalar) {
        // 純量乘法
        return new Vector(scalar * getX(), scalar * getY(), scalar * getZ());
    }

    public Vector inv() {
        return new Vector(-getX(), -getY(), -getZ());
    }

    public Vector add(Point p) {
        return new Vector(getX() + p.getX(), getY() + p.getY(), getZ() + p.getZ());
    }

    public Vector sub(Point p) {
        return add(new Vector(p).inv());
    }

    public double dot(Point p) {
        return getX() * p.getX() + getY() * p.getY() + getZ() * p.getZ();
    }

    public Vector cross(Point p) {
        return new Vector(
            getY() * p.getZ() - p.getY() * getZ(),
            getZ() * p.getX() - p.getZ() * getX(),
            getX() * p.getY() - p.getX() * getY()
        );
    }

    public double norm() {
        return Math.sqrt(dot(this));
    }

    public Vector normalize() {
        double length = norm();
        return length == 0 ? new Vector(0, 0, 0) : mul(1.0 / length);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("Vector[x=%.3f, y=%.3f, z=%.3f]", getX(), getY(), getZ());
    }
}