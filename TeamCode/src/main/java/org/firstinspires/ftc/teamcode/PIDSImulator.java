package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.limemode.PIDController;

@TeleOp(name="PIDSimulator")
public class PIDSImulator extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double target = 0;
    private DcMotorEx motor;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        PIDController controller = new PIDController(target, kP, kI, kD, multipleTelemetry);

        motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        waitForStart();
        double position = 0;
        while (opModeIsActive()) {
            multipleTelemetry.addLine("Running Simulator...");

            controller.update(position, multipleTelemetry);
            controller.sync(kP, kI, kD, target);
            multipleTelemetry.update();
        }
    }
}
