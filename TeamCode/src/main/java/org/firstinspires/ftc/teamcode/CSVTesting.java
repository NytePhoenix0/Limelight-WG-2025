package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

@TeleOp(name = "CSV", group = "Testing")
public class CSVTesting extends LinearOpMode {
    public static final String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/PIDTuning/";

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addData("Status", "Writing file...");
        telemetry.update();

        File directory = new File(PATH);
        if (!directory.exists()) {
            if (!directory.mkdirs()) {
                throw new RuntimeException("Failed to create directory: " + PATH);
            }
        }

        try {
            FileOutputStream fileOutputStream = new FileOutputStream(PATH + "test.csv");
            ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream);

            objectOutputStream.writeObject("kP,kI,kD,Time,Jerk");
            objectOutputStream.flush();
            objectOutputStream.writeObject("0.006,0.00055,0.00002,1521.5,0.5");
            objectOutputStream.flush();
            objectOutputStream.writeObject("0.0061,0.00054,0.000021,1520,0.51");
            objectOutputStream.flush();
            objectOutputStream.writeObject("0.00621,0.00056,0.000022,1522,0.51");
            objectOutputStream.flush();
            objectOutputStream.writeObject("0.0061,0.00054,0.000021,1520,0.51");
            objectOutputStream.flush();
            objectOutputStream.close();
        } catch (IOException exception) {

        }

        telemetry.addData("Status", "File saved to \"" + PATH + "\"");
        telemetry.update();
    }
}
