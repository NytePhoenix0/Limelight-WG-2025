package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.limemode.PIDController;

@TeleOp(name = "PIDTesting")
@Config
public class PIDTesting extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double target = 0;

    private DcMotorEx motor;

    private double _kP, _kI, _kD, _target;
    MultipleTelemetry multipleTelemetry;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "leftFront");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        PIDController controller = new PIDController(target, kP, kI, kD, multipleTelemetry);
        waitForStart();
        runPID();
        while (opModeIsActive()) {
            controller.sync(target, kD, kI, kP);

            if (kP != _kP || kI != _kI || kD != _kD || target != _target) {
                runPID();

                multipleTelemetry.addLine("Done!");
                multipleTelemetry.update();
            }
        }
    }

    public static boolean continueRunning = false;
    private void runPID() {
        continueRunning = true;
        telemetry.addLine("running");
        telemetry.update();
        _kP = kP;
        _kI = kI;
        _kD = kD;
        _target = target;
        double integralSum = 0;
        double lastError = 0;
        double error = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double position = 0;

        ElapsedTime timer = new ElapsedTime();

        while (Math.abs(_target - position) > 0.5 && opModeIsActive() && continueRunning) {
            position = motor.getCurrentPosition();
            error = _target - position;

            double derivative = (error - lastError) / timer.seconds();

            integralSum += error * timer.seconds();

            double output = _kP * error + _kI * integralSum + _kD * derivative;
            output /= 10;

            output = Range.clip(output, -1, 1);

            motor.setPower(output);

            multipleTelemetry.addData("kP", _kP);
            multipleTelemetry.addData("kI", _kI);
            multipleTelemetry.addData("kD", _kD);
            multipleTelemetry.addData("Target", target);
            multipleTelemetry.addData("Position", position);
            multipleTelemetry.addData("Error", error);
            multipleTelemetry.addData("Last Error", lastError);
            multipleTelemetry.addData("Integral", integralSum);
            multipleTelemetry.addData("Derivative", derivative);
            multipleTelemetry.addData("Output", output);
            multipleTelemetry.update();

            lastError = error;
            timer.reset();
        }
    }
}
