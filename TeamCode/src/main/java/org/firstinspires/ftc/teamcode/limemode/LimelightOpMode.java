package org.firstinspires.ftc.teamcode.limemode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class LimelightOpMode extends OpMode {
    protected DcMotorEx leftFront, leftRear, rightFront, rightRear;
    protected IMU imu;
    protected Limelight3A limelight;
    protected double yawOffset = 0;
    @Override
    public void init() {
        start();
        if (leftFront == null || leftRear == null || rightFront == null || rightRear == null) {
            throw new RuntimeException("A motor is not properly initialized !");
        }
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        imu.resetYaw();
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            yawOffset = result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
        }
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
        double yaw = Numbers.normalizeAngle(yawDeg);
        double yawRad = orientation.getYaw(AngleUnit.RADIANS);
        _getLimelightData(yaw);
        update();
        telemetry.update();
    }

    private void _getLimelightData(double yaw) {
        double limelightOrientation = (yaw + yawOffset) % 360;
        if (limelightOrientation > 180) limelightOrientation -= 360;
        limelight.updateRobotOrientation(limelightOrientation);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            Pose3D botposemt1 = result.getBotpose();
            telemetry.addData("MT2: position", botpose.getPosition());
            telemetry.addData("MT2: orientation", botpose.getOrientation());
            telemetry.addData("MT1: position", botposemt1.getPosition());
            telemetry.addData("MT1: orientation", botposemt1.getOrientation());
        }
        else {
            telemetry.addLine("Null pose result");
        }


    }

    public abstract void start();
    public abstract void update();


}
