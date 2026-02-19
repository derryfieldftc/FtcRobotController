package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

@TeleOp
@Disabled
public class LiftSwitchTest extends OpMode {
    TouchSensor sensor;

    @Override
    public void init() {
        sensor = hardwareMap.touchSensor.get(RobotPart.Part.LiftSwitch.name);
    }

    @Override
    public void loop() {
        telemetry.addData("state", sensor.isPressed());
        telemetry.update();
    }
}
