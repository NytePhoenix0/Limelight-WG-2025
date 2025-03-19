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
import org.firstinspires.ftc.teamcode.limemode.PIDController;

@Config
@TeleOp(name="GoToOrigin")
public class GoToOrigin extends LinearOpMode {
    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double SPEED = 1000;
    public static double SPIN_SPEED = 1000;
    public static boolean PAUSED = false;

    public static double kP = 1;
    public static double kI = 0.00000000001;
    public static double kD = 0.1;
    public MultipleTelemetry multipleTelemetry;
    public FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Chassis chassis = new Chassis(this);
        PIDController xController = new PIDController(TARGET_X, kD, kI, kD, multipleTelemetry);
        PIDController yController = new PIDController(TARGET_Y, kP, kI, kD, multipleTelemetry);
        double staringYaw = 0;
        while (!isStarted()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                YawPitchRollAngles mt1 = result.getBotpose().getOrientation();
                double y = mt1.getYaw(AngleUnit.DEGREES);
                telemetry.addData("yaw", y);
                staringYaw = y * 0.1 + staringYaw * 0.9;
            }
            telemetry.addLine("Waitinxg for start");
            telemetry.update();
        }
        chassis.yawOffset = staringYaw;
        waitForStart();
        double lastX = 0;
        double lastY = 0;
        double lastTime = -1;
        boolean wasPaused = false;
        while (opModeIsActive()) {
            // Sync values
            xController.sync(TARGET_X, kD, kI, kP);
            yController.sync(TARGET_Y, kD, kI, kP);
            if (PAUSED) {
                wasPaused = true;
                telemetry.addLine("currently paused!");
                telemetry.update();
                chassis.move(0, 0, 0, 0, 0);
                continue;
            }
            else if (wasPaused) {
                wasPaused = false;
                xController.reset();
                yController.reset();
            }
            multipleTelemetry.addLine("For graphing purposes");
            multipleTelemetry.addData("Target Y", TARGET_Y);
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);

            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                Position pos = result.getBotpose_MT2().getPosition();
                Position mt1 = result.getBotpose().getPosition();
                double x_diff = TARGET_X - pos.x;
                double y_diff = TARGET_Y - pos.y;
                multipleTelemetry.addData("Current Y", pos.y);
                multipleTelemetry.addData("Y Error", y_diff);

                multipleTelemetry.addData("X Diff", x_diff);
                multipleTelemetry.addData("Y Diff", y_diff);
                multipleTelemetry.addData("Position", pos);
                multipleTelemetry.addData("mt1", mt1);
                multipleTelemetry.addData("Speed", SPEED);

                double x = xController.update(pos.x, false);
                double y = yController.update(pos.y, true);
                multipleTelemetry.addData("moveX", x);
                multipleTelemetry.addData("moveY", y);
                chassis.move(yawRads, x, y, 0, SPEED);
                lastX = x;
                lastY = y;
                lastTime = System.currentTimeMillis();
            }
            else {
                multipleTelemetry.addLine("Cannot see apriltags, spinning");
                chassis.move(yawRads, 0, 0, 1, SPIN_SPEED);
            }
            multipleTelemetry.update();
        }
    }
}
