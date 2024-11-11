package org.firstinspires.ftc.teamcode.plugins;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.util.ArrayList;


/**
 * ServoDebug
 */
public class ServoDebug extends RobotPlugin {

    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;
    ArrayList<Servo> servos = new ArrayList<>();
    Gamepad gamepad;

    /**
     * creates a new ServoDebug
     */
    public ServoDebug(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.gamepad = opMode.gamepad1;
    }

    /**
     * adds a servo to the list of servos to debug
     * @param name
     */
    public void addServo(String name) {
        servos.add(hardwareMap.servo.get(name));
    }

    @Override
    public void init() {
        for (int i = 0; i < servos.size(); i++) {
            Servo servo = servos.get(i);
            Servo.Direction servoDirection = servo.getDirection();
            double servoPosition = servo.getPosition();
            String line = "servo " + i + " direction: " + servoDirection + " position: " + servoPosition;
            telemetry.addLine(line);
        }
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad.right_stick_y) > .3) {
            for (int i = 0; i < servos.size(); i++) {
                servos.get(i).setPosition(servos.get(i).getPosition()-gamepad.right_stick_y);
            }

        }
        for (int i = 0; i < servos.size(); i++) {
            Servo servo = servos.get(i);
            Servo.Direction servoDirection = servo.getDirection();
            double servoPosition = servo.getPosition();
            String line = "servo " + i + " direction: " + servoDirection + " position: " + servoPosition;
            telemetry.addLine(line);
        }
        telemetry.update();
    }
}