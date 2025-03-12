package org.firstinspires.ftc.teamcode.limemode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.sql.Time;
public abstract class LimelightOpMode extends OpMode {
    protected DcMotorEx leftFront, leftRear, rightFront, rightRear;
    protected IMU imu;
    protected Limelight3A limelight;
    protected double yawOffset = 0;
    protected FtcDashboard dashboard;
    private Pose3D megaTag1Pose;
    private Pose3D megaTag2Pose;
    private long tagAquisitionTime = 0;
    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
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
        // _getLimelightData(yaw);
        update();
        telemetry.update();
    }



    public Pose3D getPose() {
        return megaTag2Pose;
    }

    public abstract void start();
    public abstract void update();


}
