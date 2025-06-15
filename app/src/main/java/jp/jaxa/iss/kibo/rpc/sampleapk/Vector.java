package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Vec3d;

public class Vector extends gov.nasa.arc.astrobee.types.Point {
    public Vector(double x, double y, double z) {
        super(x, y, z);
    }

    public Vector(Point p) {
        this(p.getX(), p.getY(), p.getZ());
    }

    public Vector(Vec3d p) {
        this(p.toArray()[0], p.toArray()[1], p.toArray()[2]);
    }

    public Vector scale(double k) {
        return new Vector(k * getX(), k * getY(), k * getZ());
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
        double lenth = norm();
        return lenth == 0 ? new Vector(0, 0, 0) : scale(1.0 / lenth);
    }
}
