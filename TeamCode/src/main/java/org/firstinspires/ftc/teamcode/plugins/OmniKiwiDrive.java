package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.lib.GamepadManager;
import org.firstinspires.ftc.teamcode.lib.MotorManager;

public class OmniKiwiDrive extends RobotPlugin{
	OpMode opMode;
	GamepadManager gamepadManager;
	Gamepad gamepad;
	MotorManager front;
	MotorManager left;
	MotorManager right;

	public OmniKiwiDrive(OpMode opMode) {
		this.opMode = opMode;
		MotorManager.setGlobalOpMode(opMode);
		gamepadManager = new GamepadManager(opMode.gamepad1);
		gamepad = opMode.gamepad1;
	}

	public void init() {
		front = new MotorManager("front");
		left = new MotorManager("left");
		right = new MotorManager("right");
	}


	public void loop() {
		double frontPower, leftPower, rightPower;
		double halfSqrt3 = 0.866025403784;
		leftPower = .5 * gamepad.left_stick_x + halfSqrt3 * gamepad.left_stick_y - gamepad.right_stick_x;
		rightPower = .5 * gamepad.left_stick_x - halfSqrt3 * gamepad.left_stick_y - gamepad.right_stick_x;
		frontPower = -gamepad.left_stick_x - gamepad.right_stick_x;

		left.setPower(leftPower);
		right.setPower(rightPower);
		front.setPower(frontPower);
	}
}
