package org.firstinspires.ftc.teamcode.limemode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SimplePIDController {
    public double kP, kI, kD, target;
    private double previousTime = -1;
    private double lastError = -1;
    private double integralError = 0;
    private double actualError = 0;
    private double averageError = 0;
    public boolean firstInit = true;
    private Telemetry telemetry;
    private ElapsedTime timer;

    public SimplePIDController(double initalTarget, double kP, double kI, double kD, Telemetry debug) {
        this.kP = kP;
        this.telemetry = debug;
        this.kI = kI;
        this.kD = kD;
        this.target = initalTarget;
        timer = new ElapsedTime();
    }

    public SimplePIDController(double initalTarget, double kP, double kI, double kD) {
        this(initalTarget, kP, kI, kD, null);
    }
    public double update(double currentPos) {
        return update(currentPos, null);
    }
    public double update(double currentPos, MultipleTelemetry debug) {  
        double error = target - currentPos;
        double derivitiveError = timer.seconds() == 0 ? 0 : (error - lastError)/timer.seconds();
        integralError += error * timer.seconds();

        lastError = error;
        previousTime = timer.seconds();

        if (debug != null) {
            telemetry.addData("Error", error);
            telemetry.addData("Porportion", kP * error);
            telemetry.addData("Derivitive", kD * derivitiveError);
            telemetry.addData("Integral", kI * integralError);
            telemetry.addData("Target Position", target);
            telemetry.addData("Actual Position", currentPos);
            telemetry.addData("Delta", timer.seconds());
        }
        timer.reset();
        return (kP * error) + (kI * integralError) + (kD * derivitiveError);
    }

    public boolean sync(double target, double kD, double kI, double kP) {
        boolean changed = false;

        if (this.kD != kD) { this.kD = kD; reset(); changed = true;}
        if (this.kI != kI) { this.kI = kI; reset(); changed = true;}
        if (this.kP != kP) { this.kP = kP; reset(); changed = true;}
        if (this.target != target) { this.target = target; reset(); changed = true;}
        timer.reset();
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
