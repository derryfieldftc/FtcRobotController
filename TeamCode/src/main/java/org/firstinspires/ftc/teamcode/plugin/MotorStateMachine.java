package org.firstinspires.ftc.teamcode.plugin;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class MotorStateMachine {

    public Servo[] servos;
    public DcMotor[] motors;
    Map<String, StateAction[]> states;
    String currentState;

    public static class Builder {

        List<String> servoNames;
        List<Consumer<Servo>> servoConfigs;
        List<String> motorNames;
        List<Consumer<DcMotor>> motorConfigs;
        List<String> stateNames;
        List<List<StateActionPrototype>> stateActions;

        public Builder() {
            servoNames = new ArrayList<>();
            servoConfigs = new ArrayList<>();
            motorNames = new ArrayList<>();
            motorConfigs = new ArrayList<>();
            stateNames = new ArrayList<>();
            stateActions = new ArrayList<>();
        }
        public Builder addServo(String servoName) {
            this.addServo(servoName, servo -> {});
            return this;
        }
        public Builder addServo(String servoName, Consumer<Servo> servoConfig) {
            servoNames.add(servoName);
            servoConfigs.add(servoConfig);
            return this;
        }
        public Builder addMotor(String motorName) {
            this.addMotor(motorName, motor -> {});
            return this;
        }
        public Builder addMotor(String motorName, Consumer<DcMotor> motorConfig) {
            motorNames.add(motorName);
            motorConfigs.add(motorConfig);
            return this;
        }
        public Builder addState(String stateName, StateActionPrototype... actions) {
            stateNames.add(stateName);
            stateActions.add(Arrays.asList(actions));
            return this;
        }
        public MotorStateMachine build(HardwareMap hardwareMap) {
            MotorStateMachine stateMachine = new MotorStateMachine();
            stateMachine.states = new HashMap<>();
            List<Servo> servos = new ArrayList<>();
            List<DcMotor> motors = new ArrayList<>();

            for (int i = 0; i < servoNames.size(); i++) {
                String servoName = servoNames.get(i);
                Servo servo = hardwareMap.servo.get(servoName);
                servoConfigs.get(i).accept(servo);
                servos.add(servo);
            }
            for (int i = 0; i < motorNames.size(); i++) {
                String motorName = motorNames.get(i);
                DcMotor motor = hardwareMap.dcMotor.get(motorName);
                motorConfigs.get(i).accept(motor);
                motors.add(motor);
            }

            for (int i = 0; i < stateNames.size(); i++) {
                List<StateAction> actions = new ArrayList<>();
                for (StateActionPrototype action : stateActions.get(i)) {
                    StateAction stateAction = action.build(hardwareMap);
                    actions.add(stateAction);
                }
                String stateName = stateNames.get(i);
                stateMachine.states.put(stateName, actions.toArray(new StateAction[0]));
            }

            stateMachine.servos = servos.toArray(new Servo[0]);
            stateMachine.motors = motors.toArray(new DcMotor[0]);
            return stateMachine;
        }
    }

    public void setCurrentState(String stateName) {
        StateAction[] actions = states.get(stateName);
        if (actions != null) {
            for (StateAction action : actions) {
                action.apply();
            }
            currentState = stateName;
        }
    }

    public String[] getStates() {
        return states.keySet().toArray(new String[0]);
    }
    public String getCurrentState() {
        return currentState;
    }

    public Map<String, Servo> getServos() {
        return Arrays.stream(servos).collect(Collectors.toMap(Servo::getDeviceName, s -> s));
    }

    public Map<String, DcMotor> getMotors() {
        return Arrays.stream(motors).collect(Collectors.toMap(DcMotor::getDeviceName, m -> m));
    }

    public static class StateActionPrototype {
        String name;
        Type type;

        double servoTarget;

        double motorPower;
        int motorTarget;

        public enum Type {
            SERVO_TARGET,
            MOTOR_POWER,
            MOTOR_TARGET
        }

        private StateActionPrototype(String name, Type type, double motorPower, int motorTarget, double servoTarget) {
            this.name = name;
            this.type = type;
            this.motorPower = motorPower;
            this.motorTarget = motorTarget;
            this.servoTarget = servoTarget;
        }
        public static StateActionPrototype servoTarget(String servoName, double target) {
            return new StateActionPrototype(servoName, StateActionPrototype.Type.SERVO_TARGET, 0, 0, target);
        }
        public static StateActionPrototype motorPower(String motorName, double power) {
            return new StateActionPrototype(motorName, StateActionPrototype.Type.MOTOR_POWER, power, 0, 0);
        }
        public static StateActionPrototype motorTarget(String motorName, double power, int target) {
            return new StateActionPrototype(motorName, StateActionPrototype.Type.MOTOR_TARGET, power, target, 0);
        }

        public StateAction build(HardwareMap hardwareMap) {
            if (type == Type.SERVO_TARGET) {
                Servo servo = hardwareMap.servo.get(name);
                return new StateAction((v) -> servo.setPosition(servoTarget));
            } else if (type == Type.MOTOR_POWER) {
                DcMotor motor = hardwareMap.dcMotor.get(name);
                return new StateAction((v) -> {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(motorPower);
                });
            } else if (type == Type.MOTOR_TARGET) {
                DcMotor motor = hardwareMap.dcMotor.get(name);
                return new StateAction((v) -> {
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setTargetPosition(motorTarget);
                    motor.setPower(motorPower);
                });
            } else {
                throw new IllegalStateException("unreachable");
            }
        }
    }
    public static class StateAction {
        Consumer<Void> action;
        private StateAction(Consumer<Void> action) {
            this.action = action;
        }
        public void apply() {
            action.accept(null);
        }
        public static StateActionPrototype servoTarget(String servoName, double servoTarget) {
            return StateActionPrototype.servoTarget(servoName, servoTarget);
        }
        public static StateActionPrototype motorPower(String motorName, double motorPower) {
            return StateActionPrototype.motorPower(motorName, motorPower);
        }
        public static StateActionPrototype motorTarget(String motorName, double motorPower, int motorTarget) {
            return StateActionPrototype.motorTarget(motorName, motorPower, motorTarget);
        }
    }

}
