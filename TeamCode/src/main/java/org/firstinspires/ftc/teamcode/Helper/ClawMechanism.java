package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;

public class ClawMechanism {

    public static class State {
        public String stateName;
        public double servoPosition;
        public State(String name, double pos) { stateName = name; servoPosition = pos; }
    }

    public static class Builder {
        Servo servo;
        ArrayList<State> states;
        String defaultState;
        public Builder() {
            states = new ArrayList<>();
        }

        public void setServo(Servo servo) { this.servo = servo; }
        public void setServo(HardwareMap hardwareMap, String servoName) { this.servo = hardwareMap.servo.get(servoName); }
        public void addState(String name, double servoPosition) { states.add(new State(name, servoPosition)); }
        public void setDefaultState(String stateName) { defaultState = stateName; }

        public ClawMechanism build() throws IllegalStateException {
            ClawMechanism claw = new ClawMechanism();
            if (servo == null) {
                throw new IllegalStateException("ClawMechanism requires a servo");
            } else {
                claw.servo = servo;
            }

            if (states.size() == 0) {
                throw new IllegalStateException("ClawMechanism needs at least one state");
            } else {
                claw.states = states;
            }

            if (defaultState == null) {
                claw.currentStateIndex = 0;
            } else {
                claw.setStateByName(defaultState);
            }

            return claw;
        }
    }

    Servo servo;
    ArrayList<State> states;
    int currentStateIndex;

    public State getCurrentState() {
        return states.get(currentStateIndex);
    }
    public int getCurrentStateIndex() {
        return currentStateIndex;
    }
    public void setState(int index) {
        currentStateIndex = index;
        updatePosition();
    }
    public void setStateByName(String stateName) {
        for (int i = 0; i < states.size(); i++) {
            if (states.get(i).stateName.equals(stateName)) {
                currentStateIndex = i;
                break;
            }
        }
        updatePosition();
    }

    private void updatePosition() {
        servo.setPosition(states.get(currentStateIndex).servoPosition);
    }

}
