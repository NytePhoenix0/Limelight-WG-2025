package org.firstinspires.ftc.teamcode.foxdrive;

public class FoxDriveCore {
    public FoxDriveCore() {

    }

    public Pose feedForwardTicks(double leftFront, double leftBack, double rightBack, double rightFront, double lateralMultiplier, double trackWidth) {
        double x = (leftFront + leftBack + rightBack + rightFront) * 0.25;
        double y = (-leftFront + leftBack - rightBack + rightFront) * (0.25 / lateralMultiplier);
        double r = (-leftFront - leftBack + rightBack + rightFront) * (0.25 / trackWidth);
        return new Pose(x, y, r);
    }

}
