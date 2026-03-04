package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    OpMode opMode;
    RobotVoltageSensor voltage;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        // At this point also instantiate any sub-systems of the robot. RobotVoltageSensor is used here as an example
        voltage = new RobotVoltageSensor(opMode);
    }

    public void HelloWorld() {
        opMode.telemetry.addLine("Hello World");
        opMode.telemetry.update();
    }
}