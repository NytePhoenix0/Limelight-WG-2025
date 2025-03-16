package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public static double SPIN_SPEED = 200;
    public MultipleTelemetry multipleTelemetry;
    public FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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
                telemetry.addData("X Diff", x_diff);
                telemetry.addData("Y Diff", y_diff);
                telemetry.addData("Position", pos);
                telemetry.addData("Speed", SPEED);
                chassis.move(yawRads, x_diff, y_diff, 0, SPEED);
            }
            else {
                telemetry.addLine("Cannot see apriltags, spinning");
                chassis.move(yawRads, 0, 0, 1, SPIN_SPEED);
            }
            telemetry.update();
        }
    }
}
