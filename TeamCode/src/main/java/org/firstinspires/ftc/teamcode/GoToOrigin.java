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
    private int reached_destination = 0;
    public static int CORRECT_OFFSET_NEEDED = 200;
    public static int DESTINATION_THRESHOLD = 200;
    public static double TARGET_PRECISION = 0.05;
    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double SPEED = 1000;
    public static double SPIN_SPEED = 1000;
    public static boolean PAUSED = false;
    public static float POSITION_OFFSET_DAMPING = 50;
    public static float ROTATION_OFFSET_DAMPING = 10;

    public static double kP = 1;
    public static double kI = 0.00000000001;
    public static double kD = 0.1;
    public MultipleTelemetry multipleTelemetry;
    public FtcDashboard dashboard;
    int correct_offset_calculation = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Chassis chassis = new Chassis(this);
        PIDController xController = new PIDController(TARGET_X, kD, kI, kD, multipleTelemetry);
        PIDController yController = new PIDController(TARGET_Y, kP, kI, kD, multipleTelemetry);
        double staringYaw = 0;
        double startingYawRad = 0;

        double x_offset = 0;
        double y_offset = 0;
        while (!isStarted()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                YawPitchRollAngles mt1 = result.getBotpose().getOrientation();
                Position pos = result.getBotpose().getPosition();
                double y = mt1.getYaw(AngleUnit.DEGREES);
                telemetry.addData("yaw", y);
                telemetry.addData("startingyaw", staringYaw);
                telemetry.addData("mt1", mt1);
                double yawOffset = ((y - staringYaw + 180) % 360) - 180;
                staringYaw += yawOffset / ROTATION_OFFSET_DAMPING;
                if (Double.isNaN(staringYaw)) staringYaw = 0;
            }
            telemetry.addLine("Waitinxg for start");
            telemetry.update();
        }
        startingYawRad = (staringYaw/360) * 2 * Math.PI;
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
                reached_destination = 0;
            }
            if (reached_destination > DESTINATION_THRESHOLD) {
                telemetry.addLine("Reached destination!");
                telemetry.update();
                chassis.move(0, 0, 0, 0, 0);
                continue;
            }
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                Position mt1 = result.getBotpose().getPosition();
                Position mt2 = result.getBotpose_MT2().getPosition();
                multipleTelemetry.addData("mt1", mt1);
                multipleTelemetry.addData("mt2", mt2);
                double xdiff = mt1.x - mt2.x;
                double ydiff = mt1.y - mt2.y;
                if (correct_offset_calculation < CORRECT_OFFSET_NEEDED) {
                    x_offset += (xdiff - x_offset) / POSITION_OFFSET_DAMPING;
                    y_offset += (ydiff - y_offset) / POSITION_OFFSET_DAMPING;
                    if (Math.pow(xdiff - x_offset, 2) + Math.pow(ydiff - y_offset, 2) < TARGET_PRECISION) correct_offset_calculation++;
                    else correct_offset_calculation--;
                    multipleTelemetry.addData("offset", String.format("(%s, %s)", x_offset, y_offset));
                    multipleTelemetry.addData("correctOffset", correct_offset_calculation);
                }
                else {
                    multipleTelemetry.addData("Final offset", String.format("(%s, %s)", x_offset, y_offset));
                }
                double x_pos = mt2.x + x_offset;
                double y_pos = mt2.y + y_offset;
                multipleTelemetry.addData("Position", String.format("(%s, %s)", x_pos, y_pos));

                double x = xController.update(x_pos, false);
                double y = yController.update(y_pos, false);
                if (Math.sqrt(x * x + y * y) < TARGET_PRECISION) {
                    reached_destination += 1;
                }
                chassis.move(yawRads + startingYawRad, x, y, 0, SPEED);
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
