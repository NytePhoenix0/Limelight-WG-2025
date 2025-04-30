package org.firstinspires.ftc.teamcode.foxdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Fomx")
public class Fomx extends LinearOpMode {
    public static boolean paused = false;

    private double[][] target_positions = {
            {0, 0},
            {1.2, -1.2},
            {1.2, 1.2},
            {-1.2, 1.2},
            {-1.2, -1.2}
    };
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
