package org.firstinspires.ftc.teamcode.autotuning;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ImprovedPIDController {
    public double kP, kI, kD, target;

    private double lastError = 0;
    private double integralError = 0;
    private double lastDeriv = 0;

    private static final double SMOOTHING_ALPHA = 0.1;
    private static final double DERIVATIVE_THRESHOLD = 2;

    private Telemetry telemetry;
    private ElapsedTime timer;
    private double previousTime;

    public ImprovedPIDController(double initialTarget, double kP, double kI, double kD, Telemetry debug) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.target = initialTarget;
        this.telemetry = debug;
        this.timer = new ElapsedTime();
        this.previousTime = timer.milliseconds();
    }

    public ImprovedPIDController(double initalTarget, double kP, double kI, double kD) {
        this(initalTarget, kP, kI, kD, null);
    }

    public double update(double currentPos) {
        return update(currentPos, null);
    }

    public double update(double currentPos, MultipleTelemetry debug) {
        double currentTime = timer.milliseconds();
        double deltaTime = (currentTime - previousTime) / 1000.0;
        if (deltaTime <= 0) deltaTime = 1E-3;

        double error = target - currentPos;

        double rawDeriv = (error - lastError) / deltaTime;
        double derivitiveError = SMOOTHING_ALPHA * rawDeriv + (1 - SMOOTHING_ALPHA) * lastDeriv;
        lastDeriv = derivitiveError;

        integralError += error * deltaTime;
        lastError = error;
        previousTime = currentTime;

        if (debug != null) {
            debug.addData("Error", error);
            debug.addData("Proportional", kP * error);
            debug.addData("Integral", kI * integralError);
            debug.addData("Derivative", kD * derivitiveError);
            debug.addData("Target Position", target);
            debug.addData("Actual Position", currentPos);
            debug.addData("Delta Time", deltaTime);
        }

        return (kP * error) + (kI * integralError) + (kD * derivitiveError);
    }

    public boolean isSettled(double currentPos, double tolerance) {
        double error = target - currentPos;
        double errorRate = Math.abs(lastDeriv);
        return Math.abs(error) < tolerance && errorRate < DERIVATIVE_THRESHOLD;
    }

    public boolean sync(double target, double kD, double kI, double kP) {
        boolean changed = false;

        if (this.kD != kD) { this.kD = kD; changed = true; }
        if (this.kI != kI) { this.kI = kI; changed = true; }
        if (this.kP != kP) { this.kP = kP; changed = true; }
        if (this.target != target) { this.target = target; changed = true; }
        if (changed) reset();
        return changed;
    }

    public void reset() {
        lastError = 0;
        integralError = 0;
        lastDeriv = 0;
        previousTime = timer.milliseconds();
    }
}
