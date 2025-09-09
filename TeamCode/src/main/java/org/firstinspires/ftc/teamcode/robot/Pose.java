package org.firstinspires.ftc.teamcode.robot;

/*
 * Create our own Pose class so we can modify the values.
 * The Pose2D class has final (constant value) member variables.
 */
public class Pose {
    public double x;
    public double y;
    public double theta;

    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}
