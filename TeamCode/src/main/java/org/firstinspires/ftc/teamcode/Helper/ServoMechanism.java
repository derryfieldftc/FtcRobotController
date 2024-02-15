package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class ServoMechanism {

    // create servo state class
    public static class State {
        public String stateName;
        public double servoPosition;
        public State(String name, double pos) { stateName = name; servoPosition = pos; }
    }

    // create builder
    public static class Builder {
        Servo servo;
        ArrayList<State> states;
        String defaultState;
        public Builder() {
            states = new ArrayList<>();
        }

        public Builder setServo(Servo servo) {
            this.servo = servo;
            return this;
        }
        public Builder setServo(HardwareMap hardwareMap, String servoName) {
            this.servo = hardwareMap.servo.get(servoName);
            return this;
        }
        public Builder addState(String name, double servoPosition) {
            states.add(new State(name, servoPosition));
            return this;
        }
        public Builder setDefaultState(String stateName) {
            defaultState = stateName;
            return this;
        }

        public ServoMechanism build() throws IllegalStateException {
            ServoMechanism servoMechanism = new ServoMechanism();
            if (servo == null) {
                throw new IllegalStateException("ClawMechanism requires a servo");
            } else {
                servoMechanism.servo = servo;
            }

            if (states.size() == 0) {
                throw new IllegalStateException("ClawMechanism needs at least one state");
            } else {
                servoMechanism.states = states;
            }

            if (defaultState == null) {
                servoMechanism.currentStateIndex = 0;
            } else {
                servoMechanism.setStateByName(defaultState);
            }

            return servoMechanism;
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
