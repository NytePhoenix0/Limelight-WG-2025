package org.firstinspires.ftc.teamcode.autotuning;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name = "CSV", group = "Testing")
public class CSVTesting extends LinearOpMode {
    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/PIDTuning/";

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addData("Status", "Writing file...");
        telemetry.update();

        TuningCSV file = new TuningCSV(PATH, "test1");
        file.addData(0.1, 0.01, 0.001, 153, 0.45);
        file.addData(0.11, 0.09, 0.002, 150, 0.43);
        file.addData(0.12, 0.0098, 0.012, 141, 0.41);
        file.addData(0.125, 0.00985, 0.031, 130, 0.40);
        file.addData(0.1265, 0.00991, 0.04, 115, 0.39);
        try {
            file.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Status", "File saved to \"" + PATH + "\"");
        telemetry.update();
    }
}
