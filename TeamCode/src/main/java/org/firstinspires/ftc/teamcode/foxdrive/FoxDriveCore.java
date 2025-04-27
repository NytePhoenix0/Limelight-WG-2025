package org.firstinspires.ftc.teamcode.foxdrive;

public class FoxDriveCore {
    private double lateralMultiplier = 1;
    private double trackWidth = 1;

    public FoxDriveCore(double lateralMultiplier, double trackWidth) {
        this.lateralMultiplier = lateralMultiplier;
        this.trackWidth = trackWidth;
    }

    public void tick(double yaw) {

    }

    public Pose feedForwardTicks(double leftFront, double leftBack, double rightBack, double rightFront) {
        double x = (leftFront + leftBack + rightBack + rightFront) * 0.25;
        double y = (-leftFront + leftBack - rightBack + rightFront) * (0.25 / lateralMultiplier);
        double r = (-leftFront - leftBack + rightBack + rightFront) * (0.25 / trackWidth);
        return new Pose(x, y, r);
    }

}
