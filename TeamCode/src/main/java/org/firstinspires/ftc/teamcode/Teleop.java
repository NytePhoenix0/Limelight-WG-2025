package org.firstinspires.ftc.teamcode;

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

@TeleOp
public class Teleop extends OpMode {
    private final static double MOTOR_SPEED = 13406.4;
    DcMotorEx leftFront, leftRear, rightFront, rightRear;
    IMU imu;
    Limelight3A limelight;
    @Override
    public void init() {
        leftFront = getDriveMotor("leftFront");
        leftRear = getDriveMotor("leftRear");
        rightFront = getDriveMotor("rightFront");
        rightRear = getDriveMotor("rightRear");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double yawRads = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-yawRads) - y * Math.sin(-yawRads);
        double rotY = x * Math.sin(-yawRads) + y * Math.cos(-yawRads);
        rotX *= 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        telemetry.addData("yawRags", yawRads);
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);
        leftFront.setVelocity(frontLeftPower * MOTOR_SPEED * 0.25);
        leftRear.setVelocity(backLeftPower * MOTOR_SPEED * 0.25);
        rightFront.setVelocity(frontRightPower * MOTOR_SPEED * 0.25);
        rightRear.setVelocity(backRightPower * MOTOR_SPEED * 0.25);
        telemetry.addData("frontLeftVelocity", leftFront.getVelocity());
        telemetry.addData("backLeftVelocity", leftRear.getVelocity());
        telemetry.addData("frontRightVelocity", rightFront.getVelocity());
        telemetry.addData("backRightVelocity", rightRear.getVelocity());


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", botpose.toString());
        }
        telemetry.update();
    }

    public DcMotorEx getDriveMotor(String name) {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}
