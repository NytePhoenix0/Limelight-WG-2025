package org.firstinspires.ftc.teamcode.foxdrive;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.limemode.LimelightOpMode;

@TeleOp(name="OdometryTesting")
public class OdometryTesting extends LimelightOpMode {
    public static double MOTOR_SPEED = 2681.28;
    private final static int[] validIDs = {11,12,13,14,15,16};
    FtcDashboard dashboard;
    boolean a = false;
    @Override
    public void start() {
        leftFront = getDriveMotor("leftFront");
        leftRear = getDriveMotor("leftRear");
        rightFront = getDriveMotor("rightFront");
        rightRear = getDriveMotor("rightRear");
    }
    public static double WHEEL_RADIUS = 0.075;
    public static double GEAR_RATIO = 1/(3.61 * 5.23);
    public static double TICKS_PER_REV = 28;
    private double DISTANCE_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS)/TICKS_PER_REV;
    private double x = 0;
    private double y = 0;
    private double TLR = 0;
    private double TLF = 0;
    private double TRR = 0;
    private double TRF = 0;
    @Override
    public void update() {
        if (!a) {
            resetRuntime();
            a = true;
        }
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Yaw", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Yaw vel", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
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

        double ticksLF = leftFront.getCurrentPosition() - TLF;
        double ticksLR = leftRear.getCurrentPosition() - TLR;
        double ticksRF = rightFront.getCurrentPosition() - TRF;
        double ticksRR = rightRear.getCurrentPosition() - TRR;
        double leftFrontMeters = ticksLF * DISTANCE_PER_TICK;
        double rightFrontMeters = ticksRF * DISTANCE_PER_TICK;
        double leftRearMeters = ticksLR * DISTANCE_PER_TICK;
        double rightRearMeters = ticksRR * DISTANCE_PER_TICK;
        TLF = leftFront.getCurrentPosition();
        TLR = leftRear.getCurrentPosition();
        TRF = rightFront.getCurrentPosition();
        TRR = rightRear.getCurrentPosition();
        double deltaY = (leftFrontMeters + rightFrontMeters + leftRearMeters + rightRearMeters) / 4.0;
        double deltaX = (leftFrontMeters - rightFrontMeters - leftRearMeters + rightRearMeters) / 4.0;
        x += deltaX * cos(yawRads) - deltaY * sin(yawRads);
        y += deltaX * sin(yawRads) + deltaY * cos(yawRads);
        telemetry.addData("Predicted Position", String.format("(%s, %s)", x, y));


        telemetry.update();
    }

    public DcMotorEx getDriveMotor(String name) {
        DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }
}
