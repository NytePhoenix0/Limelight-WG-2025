package org.firstinspires.ftc.teamcode.foxdrive;

public abstract class Node {
    private final double targetX, targetY;
    // Time in seconds to wait here, -1 means do not stop
    private double stopFor = -1;
    private double leniency = 0.1;
    public Node(double x, double y, double stopFor, double threshold) {
        targetX = x;
        targetY = y;
    }
    public double getX() { return targetX; }
    public double getY() { return targetY; }
    public double getStopTime() { return stopFor; }
    public void setStopTime(double stopFor) { this.stopFor = stopFor; }
    public void setLeniency(double leniency) { this.leniency = leniency; }
}
