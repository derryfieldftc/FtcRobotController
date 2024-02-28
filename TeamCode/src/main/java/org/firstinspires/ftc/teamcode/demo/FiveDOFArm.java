package org.firstinspires.ftc.teamcode.demo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FiveDOFArm {
    /*
     * Config info
     */
    private final String baseName = "base";
    private final String shoulderName = "shoulder";
    private final String elbowName = "elbow";
    private final String wristName = "wrist";
    private final String bendName = "bend";
    private final String clawName = "claw";

    /*
     * Hide the servos to the users to protect them from getting broken.
     */
    private Servo servoBase;
    private final double BASE_MAX = 1.0;
    private final double BASE_MIN = 0.0;
    private Servo servoShoulder;
    private final double SHOULDER_MAX = 1.0;
    private final double SHOULDER_MIN = 0.0;
    private Servo servoElbow;
    private final double ELBOW_MAX = 1.0;
    private final double ELBOW_MIN = 0.0;
    private Servo servoWrist;
    private final double WRIST_MAX = 1.0;
    private final double WRIST_MIN = 0.0;
    private Servo servoBend;
    private final double BEND_MAX = 1.0;
    private final double BEND_MIN = 0.0;
    private Servo servoClaw;
    private final double CLAW_MAX = 0.5;
    private final double CLAW_MIN = 0.0;

    // Track the state of the arm.
    public enum ArmState {
        START, NEUTRAL, PREPICK, PICK, PICKUP, TURN, EXTEND, PREPLACE, PLACE
    }
    private ArmState currentState;

    /*
     * Constructor.
     */
    public FiveDOFArm (HardwareMap hardwareMap) {
        servoBase = hardwareMap.get(Servo.class, baseName);
        servoShoulder = hardwareMap.get(Servo.class, shoulderName);
        servoElbow = hardwareMap.get(Servo.class, elbowName);
        servoWrist = hardwareMap.get(Servo.class, wristName);
        servoBend = hardwareMap.get(Servo.class, bendName);
        servoClaw = hardwareMap.get(Servo.class, clawName);

        currentState = ArmState.START;
    }

    public double getBasePos() {
        return servoBase.getPosition();
    }

    public void setBasePos(double tgtPos) {
        if (tgtPos > BASE_MAX) {
            tgtPos = BASE_MAX;
        }
        if (tgtPos < BASE_MIN) {
            tgtPos = BASE_MIN;
        }
        servoBase.setPosition(tgtPos);
    }

    public double getShoulderPos() {
        return servoShoulder.getPosition();
    }

    public void setShoulderPos(double tgtPos) {
        if (tgtPos > SHOULDER_MAX) {
            tgtPos = SHOULDER_MAX;
        }
        if (tgtPos < SHOULDER_MIN) {
            tgtPos = SHOULDER_MIN;
        }
        servoShoulder.setPosition(tgtPos);
    }

    public double getElbowPos() {
        return servoElbow.getPosition();
    }

    public void setElbowPos(double tgtPos) {
        if (tgtPos > ELBOW_MAX) {
            tgtPos = ELBOW_MAX;
        }
        if (tgtPos < ELBOW_MIN) {
            tgtPos = ELBOW_MIN;
        }
        servoElbow.setPosition(tgtPos);
    }

    public double getWristPos() {
        return servoWrist.getPosition();
    }

    public void setWristPos(double tgtPos) {
        if (tgtPos > WRIST_MAX) {
            tgtPos = WRIST_MAX;
        }
        if (tgtPos < WRIST_MIN) {
            tgtPos = WRIST_MIN;
        }
        servoWrist.setPosition(tgtPos);
    }

    public double getBendPos() {
        return servoBend.getPosition();
    }

    public void setBendPos(double tgtPos) {
        if (tgtPos > BEND_MAX) {
            tgtPos = BEND_MAX;
        }
        if (tgtPos < BEND_MIN) {
            tgtPos = BEND_MIN;
        }
        servoBend.setPosition(tgtPos);
    }

    public double getClawPos() {
        return servoClaw.getPosition();
    }

    public void setClawPos(double tgtPos) {
        if (tgtPos > CLAW_MAX) {
            tgtPos = CLAW_MAX;
        }
        if (tgtPos < CLAW_MIN) {
            tgtPos = CLAW_MIN;
        }
        servoClaw.setPosition(tgtPos);
    }

    public ArmState getState() {
        return currentState;
    }

    public void setState(ArmState newState) {
        currentState = newState;
    }

    private void setPositions(double tgtBase, double tgtShoulder, double tgtElbow,
                              double tgtWrist, double tgtBend, double tgtClaw) {
        servoBase.setPosition(tgtBase);
        servoShoulder.setPosition(tgtShoulder);
        servoElbow.setPosition(tgtElbow);
        servoWrist.setPosition(tgtWrist);
        servoBend.setPosition(tgtBend);
        servoClaw.setPosition(tgtClaw);
    }

    public void updateArm() {
        switch(currentState) {
            case START:
                setPositions(0.5, 0.6, 0.5, 0.5, 0.5, 0.5);
                break;
            case NEUTRAL:
                setPositions(0.5, 0.6, 1.0, 0.5, 0.8, 0.5);
                break;
            case PREPICK:
                setPositions(0.5, 0.6, 1.0, 0.5, 0.2, 0.5);
                break;
            case PICK:
                setPositions(0.5, 0.6, 1.0, 0.5, 0.2, 0.0);
                break;
            case PICKUP:
                setPositions(0.5, 0.8, 1.0, 0.5, 0.0, 0.0);
                break;
            case TURN:
                setPositions(1.0, 0.8, 1.0, 0.5, 0.0, 0.0);
                break;
            case EXTEND:
                setPositions(1.0, 0.35, .5, 0.5, 0.0, 0.0);
                break;
            case PREPLACE:
                setPositions(1.0, 0.35, .5, 0.5, 0.0, 0.0);
                break;
            case PLACE:
                setPositions(1.0, 0.35, .5, 0.5, 0.0, 0.5);
                break;
        }
    }
}
