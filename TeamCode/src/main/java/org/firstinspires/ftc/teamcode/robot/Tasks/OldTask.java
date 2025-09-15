package org.firstinspires.ftc.teamcode.robot.Tasks;

import org.firstinspires.ftc.teamcode.robot.Pose;

import java.util.InputMismatchException;

public class OldTask<T> {

    TaskType type = null;
    Pose pose = null;
    long period = 0;             // delay period in msec.

    public static TaskType type(String token) {
        token = token.toUpperCase();
        if (token.equals("WAYPOINT")) {
            return TaskType.WAYPOINT;
        } else if (token.equals("DELAY")) {
            return TaskType.DELAY;
        } else {
            throw new InputMismatchException("Error: Unknown Task type! Problem Token: " + token);
        }
    }

    public OldTask(TaskType type) {
        this.type = type;
    }

    public OldTask(TaskType type, Pose pose) {
        this(type);
        if (type == TaskType.WAYPOINT) {
            this.pose = pose;
        } else {
            throw new InputMismatchException("A Pose argument must be used with a task of type WAYPOINT.");
        }
    }

    public OldTask(TaskType type, long periodMS) {
        this(type);
        if (type == TaskType.DELAY) {
            this.period = periodMS;
        } else {
            throw new InputMismatchException("A period argument must be used with a task of type DELAY.");
        }
    }

    public TaskType getType() {
        return type;
    }

    public void setType(TaskType type) {
        this.type = type;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public Pose getPose() {
        return this.pose;
    }

    public void setPeriod(long period) {
        this.period = period;
    }

    public long getPeriod() {
        return this.period;
    }

    @Override
    public String toString() {
        String s = "";
        if (type == TaskType.WAYPOINT) {
            s = String.format("WAYPOINT: %.06f, %.06f, %.06f", pose.x, pose.y, pose.theta);
        } else if (type == TaskType.DELAY) {
            s = String.format("DELAY: %d", period);
        } else {
            s = "UNKNOWN";
        }

        return s;
    }
}