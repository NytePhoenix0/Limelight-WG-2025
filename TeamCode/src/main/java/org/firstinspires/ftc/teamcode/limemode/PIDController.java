package org.firstinspires.ftc.teamcode.limemode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    public double kP, kI, kD, target;
    private double previousTime = -1;
    private double lastError = -1;
    private double integralError = 0;
    private double actualError = 0;
    private double averageError = 0;
    public boolean firstInit = false;
    private Telemetry telemetry;
    public PIDController(double initalTarget, double kP, double kI, double kD, Telemetry debug) {
        this.kP = kP;
        this.telemetry = debug;
        this.kI = kI;
        this.kD = kD;
        this.target = initalTarget;
        firstInit = true;
    }

    public double update(double currentPos, boolean debug) {
        double currentTime = System.currentTimeMillis();
        double deltaTime = previousTime == -1 ? 0 : currentTime - previousTime;
        deltaTime /= 1000;
        double trueError = target - currentPos;
        if (Math.abs(trueError - actualError) > 0.3 && !firstInit)
            actualError = trueError + (trueError - actualError) * 0.1;
        else actualError = trueError;
        if (firstInit) {
            averageError = actualError;
            firstInit = false;
        }
        else {
            averageError = averageError * 0.9 + actualError * 0.1;
        }
        double error = Math.abs(averageError) < 0.2 ? averageError : actualError;
        double derivitiveError = deltaTime == 0 ? 0 : (error - lastError)/deltaTime;
        integralError += error * deltaTime;

        lastError = error;
        previousTime = currentTime;
        if (debug) telemetry.addData("Error actual", error);
        return (kP * error) + (kI * integralError) + (kD * derivitiveError);
    }

    public boolean sync(double target, double kD, double kI, double kP) {
        boolean changed = false;
        if (this.kD != kD) { this.kD = kD; reset(); changed = true;}
        if (this.kI != kI) { this.kI = kI; reset(); changed = true;}
        if (this.kP != kP) { this.kP = kP; reset(); changed = true;}
        if (this.target != target) { this.target = target; reset(); changed = true;}
        return changed;
    }

    public void reset() {
        previousTime = 0;
        lastError = 0;
        integralError = 0;
        firstInit = true;
        actualError = 0;
        averageError = 0;
    }
}
