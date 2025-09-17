package org.firstinspires.ftc.teamcode.robot;

import java.util.InputMismatchException;

public class Task<T> {
    public enum Type {
        WAYPOINT,
        DELAY
    }

    Type type = null;
    Pose pose = null;
    long period = 0;             // delay period in msec.

    public static Type type(String token) {
        token = token.toUpperCase();
        if (token.equals("WAYPOINT")) {
            return Type.WAYPOINT;
        } else if (token.equals("DELAY")) {
            return Type.DELAY;
        } else {
            throw new InputMismatchException("Error: Unknown Task type! Problem Token: " + token);
        }
    }

    public Task(Type type) {
        this.type = type;
    }

    public Task(Type type, Pose pose) {
        this(type);
        if (type == Type.WAYPOINT) {
            this.pose = pose;
        } else {
            throw new InputMismatchException("A Pose argument must be used with a task of type WAYPOINT.");
        }
    }

    public Task(Type type, long periodMS) {
        this(type);
        if (type == Type.DELAY) {
            this.period = periodMS;
        } else {
            throw new InputMismatchException("A period argument must be used with a task of type DELAY.");
        }
    }

    public Type getType() {
        return type;
    }

    public void setType(Type type) {
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
        if (type == Type.WAYPOINT) {
            s = String.format("WAYPOINT: %.06f, %.06f, %.06f", pose.x, pose.y, pose.theta);
        } else if (type == Type.DELAY) {
            s = String.format("DELAY: %d", period);
        } else {
            s = "UNKNOWN";
        }

        return s;
    }
}