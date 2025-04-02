package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.limemode.LimelightOpMode;

@TeleOp
@Config
public class Teleop extends LimelightOpMode {
    public static double MOTOR_SPEED = 2681.28;
    private final static int[] validIDs = {11,12,13,14,15,16};
    FtcDashboard dashboard;
    IMU control_hub_imu;
    boolean a = false;
    @Override
    public void start() {
        leftFront = getDriveMotor("leftFront");
        leftRear = getDriveMotor("leftRear");
        rightFront = getDriveMotor("rightFront");
        rightRear = getDriveMotor("rightRear");

        control_hub_imu = hardwareMap.get(IMU.class, "imu2");
        control_hub_imu.resetYaw();
    }

    @Override
    public void update() {
        if (!a) {
            resetRuntime();
            a = true;
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        YawPitchRollAngles orientation_control = control_hub_imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw vel", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        telemetry.addData("Control Yaw", orientation_control.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Control Yaw vel", control_hub_imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        telemetry.addData("Runtime", getRuntime());

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double yawRads = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-yawRads) - y * Math.sin(-yawRads);
        double rotY = x * Math.sin(-yawRads) + y * Math.cos(-yawRads);
        rotX *= 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        telemetry.addData("yawRags", yawRads);
        leftFront.setVelocity(frontLeftPower * MOTOR_SPEED);
        leftRear.setVelocity(backLeftPower * MOTOR_SPEED);
        rightFront.setVelocity(frontRightPower * MOTOR_SPEED);
        rightRear.setVelocity(backRightPower * MOTOR_SPEED);
        telemetry.update();
    }

    public DcMotorEx getDriveMotor(String name) {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}
