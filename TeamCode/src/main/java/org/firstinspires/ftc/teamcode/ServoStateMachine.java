package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class ServoStateMachine {

    Map<String, Servo> servos;
    Map<String, Map<Servo, Float>> states;
    String currentState;

    public static class Builder {
        ArrayList<String> servoNames;
        ArrayList<Consumer<Servo>> servoConfigs;
        Map<String, Map<String, Float>> states;

        public Builder() {
            servoNames = new ArrayList<>();
            servoConfigs = new ArrayList<>();
            states = new HashMap<>();
        }


        public Builder addServo(String servoName, Consumer<Servo> servoConfig) {
            servoNames.add(servoName);
            servoConfigs.add(servoConfig);
            return this;
        }

        public Builder addState(String stateName, String[] servoNames, float[] servoValues) {
            Map<String, Float> map = new HashMap<>();
            for (int i = 0; i < servoNames.length && i < servoValues.length; i++) {
                map.put(servoNames[i], servoValues[i]);
            }
            states.put(stateName, map);
            return this;
        }

        public ServoStateMachine build(OpMode opMode) {
            HardwareMap.DeviceMapping<Servo> hardwareMap = opMode.hardwareMap.servo;


            Map<String, Servo> servos = new HashMap<>();
            for (int i = 0; i < servoNames.size(); i++) {
                Servo servo = hardwareMap.get(servoNames.get(i));
                servoConfigs.get(i).accept(servo);
                servos.put(servoNames.get(i), servo);
            }

            Map<String, Map<Servo, Float>> statesConv = new HashMap<>();
            for (String stateName : states.keySet()) {
                Map<String, Float> theoreticalServoPositions = states.get(stateName);
                Map<Servo, Float> servoPositions = new HashMap<>();
                for (String servoName : theoreticalServoPositions.keySet()) {
                    servoPositions.put(servos.get(servoName), theoreticalServoPositions.get(servoName));
                }
                statesConv.put(stateName, servoPositions);
            }

            ServoStateMachine stateMachine = new ServoStateMachine();
            stateMachine.servos = servos;
            stateMachine.states = statesConv;
            return stateMachine;
        }
    }

    AtomicInteger count = new AtomicInteger();
    public int setCurrentState(String stateName) {
        Map<Servo, Float> stateMap = this.states.get(stateName);

        if (stateMap == null) { return -1; };

        this.currentState = stateName;

        stateMap.forEach((servo, val) -> {
            servo.setPosition(val);
            count.addAndGet(1);
        });

        return count.get();
    }

    public String getCurrentState() {
        return currentState;
    }

    public Servo getServo(String servoName) {
        return this.servos.get(servoName);
    }

    public List<String> getServoNames() {
        return this.servos.entrySet().stream().map(Map.Entry::getKey).collect(Collectors.toList());
    }

    public List<Servo> getServos() {
        return this.servos.entrySet().stream().map(Map.Entry::getValue).collect(Collectors.toList());
    }

}
