package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;

public class Frame {
    protected Vector position;
    protected Quater orientation;

    public Frame(Vector position, Quater orientation) {
        this.position = position;
        this.orientation = orientation;
    }

    public Frame(Kinematics kinematics) {
        this(new Vector(kinematics.getPosition()), new Quater(kinematics.getOrientation()));
    }

    public Vector getPosition() {
        return this.position;
    }

    public Quater getOrientation() {
        return this.orientation;
    }

    public Frame relative(Frame origin) {
        return new Frame(
            position.sub(origin.getPosition()),
            orientation.rdiv(origin.getOrientation())
        );
    }

    public Frame relative(Kinematics origin) {
        Frame originFrame = new Frame(origin);
        return new Frame(
                position.sub(originFrame.getPosition()),
                orientation.rdiv(originFrame.getOrientation())
        );
    }

    public Frame absolute(Frame target) {
        return new Frame(
            position.add(target.getPosition()),
            orientation.mul(target.getOrientation())
        );
    }

    public Frame absolute(Kinematics target) {
        Frame targetFrame = new Frame(target);
        return new Frame(
                position.add(targetFrame.getPosition()),
                orientation.mul(targetFrame.getOrientation())
        );
    }
}
