package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.limemode.SimplePIDController;

@Config
@TeleOp(name="Automatic PID Tuning")
public class AutomaticPIDTuning extends LinearOpMode {
    public static boolean pause = false;
    public static double target = 1;
    public static double SPEED = 3000;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    SimplePIDController pidController;
    Chassis chassis;
    ElapsedTime timer;
    ElapsedTime finish_time;
    boolean hasFullyStopped = false;
    double runtime = -1;

    @Override
    public void runOpMode() {
        chassis = new Chassis(this);
        timer = new ElapsedTime();
        finish_time = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        pidController = new SimplePIDController(target, kP, kI, kD, multipleTelemetry);

        waitForStart();

        boolean lastPaused = false;
        while (opModeIsActive()) {
            if (pause) {
                lastPaused = true;
                multipleTelemetry.addLine("PAUSED.");
                multipleTelemetry.addData("Output:", String.format("%s,%s,%s,%s", kP, kI, kD, runtime));

                chassis.move(0, 0, 0, 0, 0);

                multipleTelemetry.update();
                continue;
            } else if (lastPaused) {
                reset();
                lastPaused = false;
            }

            pidController.sync(target, kD, kI, kP);

            double out = pidController.update(chassis.leftFront.getCurrentPosition(), multipleTelemetry);
            multipleTelemetry.addData("out", out);

            chassis.move(0, 0, out, 0, SPEED);
            if (out == 0) {
                if (!hasFullyStopped) finish_time.reset();
                hasFullyStopped = true;
                if (runtime == -1) runtime = timer.seconds();

                if (finish_time.seconds() >= 1) {
                    pause = true;
                }
            } else {
                runtime = -1;
                hasFullyStopped = false;
            }
            if (timer.seconds() > 5) {
                pause = true;
            }

            multipleTelemetry.update();
        }
    }

    public void reset() {
        chassis.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidController.reset();
        timer.reset();
        finish_time.reset();
        runtime = -1;
    }
}