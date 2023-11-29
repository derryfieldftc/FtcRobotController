package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawMechanism {

    public static final double RIGHT_CLAMPED_POSITION = 0.8;
    public static final double RIGHT_RELEASED_POSITION = 0.2;

    public static final double LEFT_CLAMPED_POSITION = 0.2;
    public static final double LEFT_RELEASED_POSITION = 0.8;

    Servo leftClaw;
    Servo rightClaw;
    State servoTargetState;

    public ClawMechanism(Servo leftClaw, Servo rightClaw) {
        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
        release();
    }

    public State getCurrentState() {
        double mu = 0.01;
        double rightPos = rightClaw.getPosition();
        double leftPos = leftClaw.getPosition();

        switch (servoTargetState) {
            case RELEASED:
                if (Math.abs(rightPos - RIGHT_CLAMPED_POSITION) > mu ||
                    Math.abs(leftPos  - LEFT_CLAMPED_POSITION) > mu) {
                    return State.CLAMPING;
                } else {
                    return State.CLAMPED;
                }
            case CLAMPED:
                if (Math.abs(rightPos - RIGHT_RELEASED_POSITION) > mu ||
                        Math.abs(leftPos  - LEFT_RELEASED_POSITION) > mu) {
                    return State.RELEASING;
                } else {
                    return State.RELEASED;
                }
            default:
                return null; // unreachable
        }
    }

    public State getTargetState() {
        return this.servoTargetState;
    }

    public void release() {
        leftClaw.setPosition(LEFT_RELEASED_POSITION);
        rightClaw.setPosition(RIGHT_RELEASED_POSITION);
        servoTargetState = State.RELEASED;
    }
    public void clamp() {
        leftClaw.setPosition(LEFT_CLAMPED_POSITION);
        rightClaw.setPosition(RIGHT_CLAMPED_POSITION);
        servoTargetState = State.CLAMPED;
    }

    public enum State {
        CLAMPED, RELEASED,
        CLAMPING, RELEASING;
    }

}
