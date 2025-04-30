package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.limemode.SimplePIDController;

@Config
@TeleOp(name="GoToSuperPosition")
public class GoToSuperPosition extends LinearOpMode {
    public static double TIME_BETWEEN_TARGETS = 0;
    int TOTAL_TARGETS = 5;
    int CURRENT_TARGET = 0;
//    private final double[] X_TARGETS = new double[] {0, 1.2 , 1.2, -1.2, -1.2};
//    private final double[] Y_TARGETS = new double[] {0, -1.2, 1.2, 1.2 , -1.2};
    private final double[] X_TARGETS = new double[] {0, 1.2 , 1.2, -1.2, -1.2};
    private final double[] Y_TARGETS = new double[] {0, -1.2, 1.2, 1.2 , -1.2};
    private double reached_destination = 0;
    public static double DESTINATION_THRESHOLD = 100;
    public static double TARGET_PRECISION = 0.05;
    public static double SLOWDOWN_PRECISION = 0.2;
    public static double SPIN_SLOWDOWN_THINGY = 500;
    public double TARGET_X = 0;
    public double TARGET_Y = 0;
    public static double SPEED = 300000;
    public static double SLOWDOWN_SPEED = 1000;
    private double currentSpeed = 1000;
    public static double SPIN_SPEED = 1000;
    public static boolean PAUSED = false;
//    public static double kP = 1;
//    public static double kI = 0.00000000001;
//    public static double kD = 0.1;
    public static double kP = 0.006;
    public static double kI = 0.00055;
    public static double kD = 0.00002;
    public MultipleTelemetry multipleTelemetry;
    public FtcDashboard dashboard;
    private AndroidSoundPool androidSoundPool;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Chassis chassis = new Chassis(this);
        SimplePIDController controller = new SimplePIDController(0, kD, kI, kD);
        boolean is_on_blue = true;
        androidSoundPool = new AndroidSoundPool();
        androidSoundPool.initialize(SoundPlayer.getInstance());
        while (!isStarted()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (result != null) {
                YawPitchRollAngles mt1 = result.getBotpose().getOrientation();
                double y = mt1.getYaw(AngleUnit.DEGREES);
                telemetry.addData("megatag yaw", y);
            }
            telemetry.addLine("Waiting for start");
            if (gamepad1.a) is_on_blue = true;
            else if (gamepad1.b) is_on_blue = false;
            telemetry.addLine("Starting on " + (is_on_blue ? "BLUE" : "RED"));
            telemetry.update();
        }
        chassis.yawOffset = is_on_blue ? 0 : 180;
        double movementOffset = (chassis.yawOffset/360) * 2 * Math.PI;
        chassis.imu.resetYaw();
        waitForStart();
        double lastX = 0;
        double lastY = 0;
        double lastTime = -1;
        boolean wasPaused = false;
        double lastDeltaTime = System.currentTimeMillis();
        double waittime = 0;
        boolean pressed = false;
        while (opModeIsActive()) {
            TARGET_X = X_TARGETS[CURRENT_TARGET%TOTAL_TARGETS];
            TARGET_Y = Y_TARGETS[CURRENT_TARGET%TOTAL_TARGETS];
            double delta = (System.currentTimeMillis() - lastDeltaTime);
            multipleTelemetry.addData("Delta", delta);
            // Sync values
            boolean synced = controller.sync(0, kD, kI, kP);
            if (synced) reached_destination = 0;

            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();
            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU Yaw", yawDeg);
            telemetry.addData("Yaw Offset", chassis.yawOffset);
            LLResult result = chassis.getLimelightData(yawDeg);
            if (gamepad1.dpad_right) {
                if (!pressed) {
                    PAUSED = !PAUSED;
                }
                pressed = true;
            }
            else if (pressed) pressed = false;
            if (PAUSED) {
                wasPaused = true;
                telemetry.addLine("currently paused!");
                if (result != null) {
                    Position mt2 = result.getBotpose_MT2().getPosition();
                    telemetry.addData("Position", mt2);
                }
                else {
                    telemetry.addLine("Cannot see apriltags to find position...");
                }
                telemetry.update();
                if (gamepad1.dpad_left) {
                    chassis.move(yawRads, 0, 0, Range.clip(yawDeg/30, -1, 1), SPIN_SPEED);
                }
                else {
                    chassis.move(yawRads, 0, 0, 0, 0);
                }
                lastDeltaTime = System.currentTimeMillis();
                continue;
            }
            else if (wasPaused) {
                wasPaused = false;
                controller.reset();
                reached_destination = 0;
            }
            if (reached_destination > DESTINATION_THRESHOLD) {
                androidSoundPool.play("RawRes:ss_laser");
                telemetry.addLine("Reached destination!");
                telemetry.update();
                chassis.move(0, 0, 0, 0, 0);
                CURRENT_TARGET++;
                controller.reset();
                reached_destination = 0;

                lastDeltaTime = System.currentTimeMillis();
                continue;
            }
            multipleTelemetry.addData("Target index", CURRENT_TARGET);
            multipleTelemetry.addData("Target position", String.format("(%s, %s)", TARGET_X, TARGET_Y));
            if (result != null) {
                Position mt2 = result.getBotpose_MT2().getPosition();
                double x_pos = mt2.x;
                double y_pos = mt2.y;
                multipleTelemetry.addData("Position", String.format("(%s, %s)", x_pos, y_pos));
                double error_x = TARGET_X - x_pos;
                double error_y = TARGET_Y - x_pos;
                double angle = Math.atan2(TARGET_Y - y_pos, TARGET_X - x_pos);
                multipleTelemetry.addData("angle", angle);
                double distance = Math.sqrt(error_x * error_x + error_y * error_y);
                double power = controller.update(distance);
                double x = Math.cos(angle);
                double y = Math.sin(angle);
                multipleTelemetry.addData("Distance", distance);
                currentSpeed = power * SPEED;
                multipleTelemetry.addData("Speed", currentSpeed);
                if (distance < TARGET_PRECISION) {
                    reached_destination += delta;
                }
                multipleTelemetry.addData("reached_destination", reached_destination);
                chassis.move(yawRads + movementOffset, x, y, 0, currentSpeed);
                lastX = x;
                lastY = y;
                lastTime = System.currentTimeMillis();
            }
            else {
                double timeDiff = (System.currentTimeMillis()-lastTime)/SPIN_SLOWDOWN_THINGY;
                double curve = Range.clip(-Math.pow(timeDiff, 2) + 1, 0, 1);
                multipleTelemetry.addData("Curve", curve);
                multipleTelemetry.addLine("Cannot see apriltags, spinning");

                chassis.move(yawRads + movementOffset, lastX * curve, lastY * curve, 1 - curve, SPIN_SPEED);
            }
            lastDeltaTime = System.currentTimeMillis();
            multipleTelemetry.update();
        }
    }
}
