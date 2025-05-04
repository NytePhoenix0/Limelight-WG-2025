package org.firstinspires.ftc.teamcode.foxdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Chassis;

@TeleOp(name="Fomx")
public class Fomx extends LinearOpMode {
    public static boolean paused = false;

    int targetIndex = 0;
    private double[][] targetPositions = {
            {0, 0},
            {1.2, -1.2},
            {1.2, 1.2},
            {-1.2, 1.2},
            {-1.2, -1.2}
    };
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(this);
        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = chassis.imu.getRobotYawPitchRollAngles();

            double yawRads = orientation.getYaw(AngleUnit.RADIANS);
            double yawDeg = orientation.getYaw(AngleUnit.DEGREES);

            if (paused) {
                chassis.move(0, 0, 0, 0, 0);
                continue;
            }

            double targetX = targetPositions[targetIndex][0];
            double targetY = targetPositions[targetIndex][1];


        }
    }
}
