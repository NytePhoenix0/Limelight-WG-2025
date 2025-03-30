package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public IMU imu;
    public Limelight3A limelight;
    public double yawOffset = 0;
    private OpMode opMode;
    public Chassis(OpMode mode) {
        opMode = mode;
        leftFront = getDriveMotor("leftFront");
        leftRear = getDriveMotor("leftRear");
        rightFront = getDriveMotor("rightFront");
        rightRear = getDriveMotor("rightRear");

        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        imu.resetYaw();
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public LLResult getLimelightData(double yaw) {
        double limelightOrientation = (yaw + yawOffset) % 360;
        if (limelightOrientation > 180) limelightOrientation -= 360;
        limelight.updateRobotOrientation(limelightOrientation);
        opMode.telemetry.addData("LimelightYaw", limelightOrientation);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result;
        }
        return null;
    }

    public void move(double yawRads, double x, double y, double r, double s) {
        double rotX = x * Math.cos(-yawRads) - y * Math.sin(-yawRads);
        double rotY = x * Math.sin(-yawRads) + y * Math.cos(-yawRads);
        rotX *= 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(r), 1);
        double frontLeftPower = (rotY + rotX + r) / denominator;
        double backLeftPower = (rotY - rotX + r) / denominator;
        double frontRightPower = (rotY - rotX - r) / denominator;
        double backRightPower = (rotY + rotX - r) / denominator;
        opMode.telemetry.addLine(String.format("Moving the robot with the following powers:"));
        opMode.telemetry.addData("Left front", frontLeftPower);
        opMode.telemetry.addData("left back", backLeftPower);
        opMode.telemetry.addData("Right front", frontRightPower);
        opMode.telemetry.addData("right back", backRightPower);
        leftFront.setVelocity(frontLeftPower * s);
        leftRear.setVelocity(backLeftPower * s);
        rightFront.setVelocity(frontRightPower * s);
        rightRear.setVelocity(backRightPower * s);
    }

    public DcMotorEx getDriveMotor(String name) {
        DcMotorEx motor = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}
