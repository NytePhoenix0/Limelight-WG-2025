package org.firstinspires.ftc.teamcode.autotuning;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.Objects;

public class TuningCSV {
    private static final String HEADER = "kP,kI,kD,time,jerk\n";

    private final String path;
    private final String name;
    private final File directory;
    private BufferedWriter writer;

    public TuningCSV(String path, String name) {
        this.path = path;
        this.name = name;

        directory = new File(path);
        if (!directory.exists()) {
            if (!directory.mkdirs()) {
                throw new RuntimeException("Failed to create directory: " + path);
            }
        }

        try {
            open();
            writeRaw(HEADER);
            flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public TuningCSV(String path, String name, Map<DataKey, DataValue> init) {
        this(path, name);
        addDataBulk(init);
        try {
            flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static void saveToFile(String path, String name, Map<DataKey, DataValue> data) {
        TuningCSV file = new TuningCSV(path, name, data);
        try {
            file.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void addData(double kP, double kI, double kD, double time, double jerk) {
        try {
            if (writer == null) {
                open();
            }

            writeRaw(kP + "," + kI + "," + kD + "," + time + "," + jerk + "\n");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void addDataLive(double kP, double kI, double kD, double time, double jerk) {
        addData(kP, kI, kD, time, jerk);
        try {
            flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void addDataBulk(Map<DataKey, DataValue> data) {
        data.forEach((key, value) -> addData(key.kP, key.kI, key.kD, value.time, value.jerk));
    }

    public void open() throws IOException {
        writer = new BufferedWriter(new FileWriter(path + name + ".csv"));
    }

    public void writeRaw(String string) throws IOException {
        writer.write(string);
    }

    public void flush() throws IOException {
        writer.flush();
    }

    public void close() throws IOException {
        writer.close();
        writer = null;
    }

    public static final class DataKey {
        private final double kP;
        private final double kI;
        private final double kD;

        DataKey(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double kP() {
            return kP;
        }

        public double kI() {
            return kI;
        }

        public double kD() {
            return kD;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            DataKey that = (DataKey) obj;
            return Double.doubleToLongBits(this.kP) == Double.doubleToLongBits(that.kP) &&
                    Double.doubleToLongBits(this.kI) == Double.doubleToLongBits(that.kI) &&
                    Double.doubleToLongBits(this.kD) == Double.doubleToLongBits(that.kD);
        }

        @Override
        public int hashCode() {
            return Objects.hash(kP, kI, kD);
        }

        @Override
        public String toString() {
            return "DataKey[" +
                    "kP=" + kP + ", " +
                    "kI=" + kI + ", " +
                    "kD=" + kD + ']';
        }


        }

    public static final class DataValue {
        private final double time;
        private final double jerk;

        DataValue(double time, double jerk) {
            this.time = time;
            this.jerk = jerk;
        }

        public double time() {
            return time;
        }

        public double jerk() {
            return jerk;
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) return true;
            if (obj == null || obj.getClass() != this.getClass()) return false;
            DataValue that = (DataValue) obj;
            return Double.doubleToLongBits(this.time) == Double.doubleToLongBits(that.time) &&
                    Double.doubleToLongBits(this.jerk) == Double.doubleToLongBits(that.jerk);
        }

        @Override
        public int hashCode() {
            return Objects.hash(time, jerk);
        }

        @Override
        public String toString() {
            return "DataValue[" +
                    "time=" + time + ", " +
                    "jerk=" + jerk + ']';
        }


        }
}
