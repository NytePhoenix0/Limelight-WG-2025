package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.limemode.PIDController;
import org.firstinspires.ftc.teamcode.limemode.SimplePIDController;

@Config
@TeleOp(name="EncoderPIDTesting")
public class EncoderPIDTesting extends LinearOpMode {
    public static double target = 1;
    public static double SPEED = 3000;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        SimplePIDController controller = new SimplePIDController(target, kP, kI, kD, multipleTelemetry);
        waitForStart();
        while (opModeIsActive()) {
            controller.sync(target, kP, kI, kD);
            double out = controller.update(chassis.leftFront.getCurrentPosition(), multipleTelemetry);
            multipleTelemetry.addData("out", out);
            chassis.move(0, 0, out, 0, SPEED);
            multipleTelemetry.update();
        }
    }
}
