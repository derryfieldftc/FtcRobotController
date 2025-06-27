package org.firstinspires.ftc.teamcode.binarybot;

import java.util.InputMismatchException;

public class Task<T> {
    public enum Type {
        PEN_UP,
        PEN_DOWN,
        WAYPOINT,
        DELAY
    }

    Type type = null;
    Pose pose = null;
    long period = 0;             // delay period in msec.

    public static Type type(String token) {
        token = token.toUpperCase();
        if(token.equals("PEN_UP")) {
            return Type.PEN_UP;
        } else if (token.equals("PEN_DOWN")) {
            return Type.PEN_DOWN;
        } else if (token.equals("WAYPOINT")) {
            return Type.WAYPOINT;
        } else if (token.equals("DELAY")) {
            return Type.DELAY;
        } else {
            throw new InputMismatchException("Error: Unknown Task type!");
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

    public Task(Type type, long period) {
        this(type);
        if (type == Type.DELAY) {
            this.period = period;
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
        if (type == Type.PEN_UP) {
            s = "PEN_UP";
        } else if (type == Type.PEN_DOWN) {
            s = "PEN_DOWN";
        } else if (type == Type.WAYPOINT) {
            s = String.format("WAYPOINT: %.06f, %.06f, %.06f", pose.x, pose.y, pose.theta);
        } else if (type == Type.DELAY) {
            s = String.format("DELAY: %d", period);
        } else {
            s = "UNKNOWN";
        }

        return s;
    }
}