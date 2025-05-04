package org.firstinspires.ftc.teamcode.foxdrive;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Chassis;
import org.firstinspires.ftc.teamcode.limemode.PIDController;
import org.firstinspires.ftc.teamcode.limemode.SimplePIDController;

@Config
@TeleOp(name="angular pid testing")
public class Fomx extends LinearOpMode {
    public static boolean paused = false;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double THRESHOLD = 0.1;
    public static double SPEED = 3000;
    public static double ROTATION_SPEED = 1000;

    public static double WHEEL_RADIUS = 2.5;
    public static double GEAR_RATIO = 0;
    public static double TICKS_PER_REV = 0;
    private double DISTANCE_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS)/TICKS_PER_REV;

    int targetIndex = 0;
    private double[][] targetPositions = {
            {0, 0},
            {1.2, -1.2},
            {1.2, 1.2},
            {-1.2, 1.2},
            {-1.2, -1.2}
    };

    private double x = 0;
    private double y = 0;
    private double currentTickLF;
    private double currentTickLR;
    private double currentTickRF;
    private double currentTickRR;

    private boolean queueReset = false;
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(this);
        SimplePIDController controller = new SimplePIDController(0, kP, kI, kD);
        boolean is_on_blue = true;
        while (!isStarted()) {
            telemetry.addLine("Waiting for start, A/B to change sides");
            if (gamepad1.a) is_on_blue = true;
            else if (gamepad1.b) is_on_blue = false;
            telemetry.addLine("Starting on " + (is_on_blue ? "BLUE" : "RED"));
            telemetry.update();
        }
        chassis.yawOffset = is_on_blue ? 0 : 180;

        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();

            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);

            if (paused) {
                chassis.move(0, 0, 0, 0, 0);
                queueReset = true;
                continue;
            }

            double targetX = targetPositions[targetIndex][0];
            double targetY = targetPositions[targetIndex][1];
            LLResult llresult = chassis.getLimelightData(yawDeg);
            if (llresult != null) {
                Position currentPosition = llresult.getBotpose_MT2().getPosition();
                x = currentPosition.x;
                y = currentPosition.z;
                currentTickLF = chassis.leftFront.getCurrentPosition();
                currentTickLR = chassis.leftRear.getCurrentPosition();
                currentTickRF = chassis.rightFront.getCurrentPosition();
                currentTickRR = chassis.rightRear.getCurrentPosition();
                telemetry.addData("Current Position", String.format("(%s, %s)", x, y));
            } else {
                // do stupid feedforward stuff here later
                telemetry.addLine("No apriltags seen!! falling back to encoder odometry...");
                double ticksLF = chassis.leftFront.getCurrentPosition() - currentTickLF;
                double ticksLR = chassis.leftRear.getCurrentPosition() - currentTickLR;
                double ticksRF = chassis.rightFront.getCurrentPosition() - currentTickRF;
                double ticksRR = chassis.rightRear.getCurrentPosition() - currentTickRR;
                double leftFrontMeters = ticksLF * DISTANCE_PER_TICK;
                double rightFrontMeters = ticksRF * DISTANCE_PER_TICK;
                double leftRearMeters = ticksLR * DISTANCE_PER_TICK;
                double rightRearMeters = ticksRR * DISTANCE_PER_TICK;
                currentTickLF = chassis.leftFront.getCurrentPosition();
                currentTickLR = chassis.leftRear.getCurrentPosition();
                currentTickRF = chassis.rightFront.getCurrentPosition();
                currentTickRR = chassis.rightRear.getCurrentPosition();
                double deltaY = (leftFrontMeters + rightFrontMeters + leftRearMeters + rightRearMeters) / 4.0;
                double deltaX = (leftFrontMeters - rightFrontMeters - leftRearMeters + rightRearMeters) / 4.0;
                x += deltaX * cos(yawRads) - deltaY * sin(yawRads);
                y += deltaX * sin(yawRads) + deltaY * cos(yawRads);
                telemetry.addData("Predicted Position", String.format("(%s, %s)", x, y));
            }
            double distX = targetX - x;
            double distY = targetY - y;
            double distance = Math.sqrt(distX * distX + distY * distY);
            if (llresult != null && llresult.isValid()) {
                if (queueReset) {
                    queueReset = false;
                    controller.reset();
                    continue;
                }
                double pow = controller.update(distance);
                chassis.move(yawRads, distX/distance, distY/distance, 0, pow * SPEED);
                if (pow < THRESHOLD) {
                    targetIndex++;
                    queueReset = true;
                }
            } else {
                queueReset = true;
                chassis.move(yawRads, 0, 0, 1, ROTATION_SPEED);
            }
        }
    }
}
