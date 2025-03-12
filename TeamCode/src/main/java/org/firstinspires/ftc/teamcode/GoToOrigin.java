package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name="GoToOrigin")
public class GoToOrigin extends LinearOpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double SPEED = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(this);
        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                Position pos = result.getBotpose_MT2().getPosition();
                double x_diff = TARGET_X - pos.x;
                double y_diff = TARGET_Y - pos.y;
                chassis.move(yawRads, x_diff * SPEED, y_diff * SPEED, 0);
            }
            else telemetry.addLine("welp i guess its over.");
            telemetry.update();
        }
    }
}
