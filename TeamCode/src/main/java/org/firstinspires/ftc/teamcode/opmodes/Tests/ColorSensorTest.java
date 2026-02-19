package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
@Disabled
public class ColorSensorTest extends OpMode {
	Robot robot;

	@Override
	public void init() {
		robot = new Robot(this);
	}

	@Override
	public void loop() {

	}
}
