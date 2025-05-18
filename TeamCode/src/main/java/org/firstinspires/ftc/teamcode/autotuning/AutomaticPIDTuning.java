package org.firstinspires.ftc.teamcode.autotuning;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.limemode.SimplePIDController;

import java.io.IOException;

@Config
@TeleOp(name="Automatic PID Tuning")
public class AutomaticPIDTuning extends LinearOpMode {
    public static final String FILE_PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/PIDTuning/";

    public static boolean runFinished = false;
    public static boolean paused = false;
    public static double target = 2000;
    public static double SPEED = 3000;
    public static double kP = 0.01;
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

        TuningCSV saveFile = new TuningCSV(FILE_PATH, "auto_tuning_data");

        waitForStart();

        boolean lastPaused = false;
        while (opModeIsActive()) {
            if (gamepad1.start) {
                paused = true;
            }

            if (runFinished) {
                chassis.move(0, 0, 0, 0, 0);
                multipleTelemetry.update();

                saveFile.addDataLive(kP, kI, kD, runtime, -1);
                target = -target;

                reset();
                runFinished = false;
            }

            if (paused) {
                lastPaused = true;
                multipleTelemetry.addLine("PAUSED.");
                multipleTelemetry.addData("Output:", String.format("%s,%s,%s,%s", kP, kI, kD, runtime));
                multipleTelemetry.update();

                chassis.move(0, 0, 0, 0, 0);

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
                if (runtime == -1) runtime = timer.milliseconds();

                if (finish_time.seconds() >= 1) {
                    runFinished = true;
                }
            } else {
                runtime = -1;
                hasFullyStopped = false;
            }
//            if (timer.seconds() > 5) {
//                runFinished = true;
//            }

            multipleTelemetry.update();
        }

        try {
            saveFile.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
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