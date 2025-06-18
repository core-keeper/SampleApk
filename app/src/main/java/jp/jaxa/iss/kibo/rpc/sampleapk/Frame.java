package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class Frame {
    private Vector position;
    private Quater orientation;

    public Frame() {
        this(new Vector(0.0, 0.0, 0.0), new Quater(0.0f, 0.0f, 0.0f, 1.0f));
    }

    public Frame(Vector position, Quater orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Frame(Kinematics kinematics) {
        this(new Vector(kinematics.getPosition()), new Quater(kinematics.getOrientation()));
    }

    public Frame(KiboRpcApi api) {
        this(api.getRobotKinematics());
    }

    public Vector getPosition() {
        return this.position;
    }

    public Quater getOrientation() {
        return this.orientation;
    }

    public Frame gain(double rate) {
        return new Frame(
            position.gain(rate),
            orientation.gain(rate)
        );
    }

    public Frame absolute(Frame target) {
        return new Frame(
            position.absolute(target.getPosition()),
            orientation.absolute(target.getOrientation())
        );
    }

    public Frame absolute(Kinematics target) {
        return this.absolute(new Frame(target));
    }

    public Frame absolute(KiboRpcApi api) {
        return this.absolute(api.getRobotKinematics());
    }

    public Frame relative(Frame origin) {
        return new Frame(
            position.relative(origin.getPosition()),
            orientation.relative(origin.getOrientation())
        );
    }

    public Frame relative(Kinematics origin) {
        return this.relative(new Frame(origin));
    }

    public Frame relative(KiboRpcApi api) {
        return this.relative(api.getRobotKinematics());
    }

    public Result moveTo(KiboRpcApi api, boolean printRobotPosition) {
        return api.moveTo(position, orientation, printRobotPosition);
    }

    public Result relativeMoveTo(KiboRpcApi api, boolean printRobotPosition) {
        // relative position, absolute orientation
        return api.relativeMoveTo(position, orientation, printRobotPosition);
    }

    public Frame anchor(KiboRpcApi api, int frequency) {
        Log.i("anchor", "start: " + new Frame(api));

        long startTime = System.currentTimeMillis();

        Frame robotFrame = new Frame();
        for (int i = 0; i < frequency; i++) {
            moveTo(api, true);
            robotFrame = robotFrame.absolute(api);

            Kinematics kinematics = api.getRobotKinematics();
            Vector v, a, w;
            v = new Vector(kinematics.getLinearVelocity());
            a = new Vector(kinematics.getLinearAcceleration());
            w = new Vector(kinematics.getAngularVelocity());
            Log.i("anchor", "v: " + v + ", a: " + a + ", w: " + w);
            Log.i("anchor", "anchoring: " + new Frame(api));
        }
        robotFrame = robotFrame.gain(1.0 / frequency);

        long endTime = System.currentTimeMillis();
        Log.i("anchor", "used_time: " + (endTime - startTime) / 1000 + " seconds.");

        Log.i("anchor", "anchor_target: " + this);
        Log.i("anchor", "anchored_robot: " + robotFrame);

        Frame errorFrame = relative(robotFrame);
        Log.i("anchor", "anchored_error: " + errorFrame);
        Log.i("anchor", "robot_current: " + new Frame(api));

        return errorFrame;
    }

    @Override
    public String toString() {
        return "Frame{ position = " + position + ", orientation = " + orientation + " }";
    }
}