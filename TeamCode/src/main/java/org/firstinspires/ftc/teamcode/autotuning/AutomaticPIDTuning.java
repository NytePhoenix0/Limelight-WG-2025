package org.firstinspires.ftc.teamcode.autotuning;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Chassis;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import java.util.Random;

@TeleOp(name = "PID AutoTuner", group = "Tuning")
public class AutomaticPIDTuning extends LinearOpMode {
    private static final double TARGET_POSITION = 2000;
    private static final double TOLERANCE = 15;
    private static final double MAX_TIME = 5.0;
    private static final double SESSION_TIMEOUT = 300.0;

    private static final double START_KP = 0.005;
    private static final double START_KI = 0;
    private static final double START_KD = 0.0005;

    private static final double MAX_KP = 0.05;
    private static final double MAX_KI = 0.01;
    private static final double MAX_KD = 0.01;

    private static final double PERTURBATION = 0.5;

    private final Random random = new Random();

    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(this);
        TuningCSV saveFile = new TuningCSV(Environment.getExternalStorageDirectory().getAbsolutePath() + "/PIDTuning/", "auto_tuning_results");

        double bestTime = Double.MAX_VALUE;
        double bestJerk = Double.MAX_VALUE;
        double bestKP = START_KP, bestKI = START_KI, bestKD = START_KD;

        ElapsedTime sessionTimer = new ElapsedTime();

        waitForStart();
        sessionTimer.reset();

        double targetPosition = -TARGET_POSITION;
        while (opModeIsActive() && sessionTimer.seconds() < SESSION_TIMEOUT) {
            double kP = randomNext(bestKP, MAX_KP);
            double kI = randomNext(bestKI, MAX_KI);
            double kD = randomNext(bestKD, MAX_KD);

            targetPosition = -targetPosition;

            ImprovedPIDController pid = new ImprovedPIDController(targetPosition, kP, kI, kD, telemetry);
            ElapsedTime timer = new ElapsedTime();
            double previousVelocity = 0;
            double totalJerk = 0;

            resetEncoders(chassis);

            timer.reset();
            while (opModeIsActive() && timer.seconds() < MAX_TIME) {
                double current = getAveragePosition(chassis);
                double power = pid.update(current);
                power = Range.clip(power, -1, 1);

                setAllDrivePower(chassis, power);

                double currentVelocity = getAverageVelocity(chassis);
                double jerk = Math.abs(currentVelocity - previousVelocity);
                previousVelocity = currentVelocity;
                totalJerk += jerk;

                telemetry.addData("kP", kP);
                telemetry.addData("kI", kI);
                telemetry.addData("kD", kD);
                telemetry.addData("Power", power);
                telemetry.addData("Error", targetPosition - current);
                telemetry.update();

                if (pid.isSettled(current, TOLERANCE) && Math.abs(currentVelocity) < 5) {
                    break;
                }
            }

            double timeTaken = timer.seconds();
            saveFile.addDataLive(kP, kI, kD, timeTaken, totalJerk);

            if (timeTaken < bestTime && totalJerk < bestJerk) {
                bestTime = timeTaken;
                bestJerk = totalJerk;
                bestKP = kP;
                bestKI = kI;
                bestKD = kD;
                telemetry.addLine("New Best Found!");
            } else {
                telemetry.addLine("Not better");
            }

            telemetry.addData("Best Time", bestTime);
            telemetry.addData("Best Jerk", bestJerk);
            telemetry.update();

            sleep(500);
        }

        try {
            saveFile.close();
            writeBestToFile(bestKP, bestKI, bestKD);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private double randomNext(double base, double max) {
        double range = max * PERTURBATION;
        double result = base + (random.nextDouble() * 2 - 1) * range;
        return Math.max(0, Math.min(result, max));
    }

    private void resetEncoders(Chassis chassis) {
        chassis.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chassis.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        chassis.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chassis.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getAveragePosition(Chassis chassis) {
        return chassis.leftFront.getCurrentPosition();
        // TODO: Test if this is worse or better than just using one wheel
//        return (
//                chassis.leftFront.getCurrentPosition() +
//                        chassis.leftRear.getCurrentPosition() +
//                        chassis.rightFront.getCurrentPosition() +
//                        chassis.rightRear.getCurrentPosition()
//        ) / 4.0;
    }

    private double getAverageVelocity(Chassis chassis) {
        return chassis.leftFront.getVelocity();
        // TODO: Test if this is worse or better than just using one wheel
//        return (
//                chassis.leftFront.getVelocity() +
//                        chassis.leftRear.getVelocity() +
//                        chassis.rightFront.getVelocity() +
//                        chassis.rightRear.getVelocity()
//        ) / 4.0;
    }

    private void setAllDrivePower(Chassis chassis, double power) {
        chassis.leftFront.setPower(power);
        chassis.leftRear.setPower(power);
        chassis.rightFront.setPower(power);
        chassis.rightRear.setPower(power);
    }

    private void writeBestToFile(double kP, double kI, double kD) throws IOException {
        FileWriter writer = new FileWriter(Environment.getExternalStorageDirectory().getAbsolutePath() + "/PIDTuning/best_pid_constants.txt");
        writer.write(String.format(Locale.ENGLISH, "kP=%.6f\nkI=%.6f\nkD=%.6f\n", kP, kI, kD));
        writer.close();
    }
}
