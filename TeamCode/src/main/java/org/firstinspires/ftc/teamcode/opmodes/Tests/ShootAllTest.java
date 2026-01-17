package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class ShootAllTest extends OpMode {
	boolean running = true;
	Robot robot;

	@Override
	public void init() {
		robot = new Robot(this);
	}

	@Override
	public void loop() {
		if (running)
			running = robot.shootAll().run();

	}
}
