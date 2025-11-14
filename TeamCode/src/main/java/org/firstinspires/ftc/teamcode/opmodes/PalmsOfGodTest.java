package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

public class PalmsOfGodTest extends OpMode {
	Servo left, right;

	@Override
	public void init() {
		left = hardwareMap.servo.get(RobotPart.Part.LeftPalm.name);
		right = hardwareMap.servo.get(RobotPart.Part.RightPalm.name);

	}

	@Override
	public void loop() {
		telemetry.addData("left", left.getPosition());
		telemetry.addData("right", right.getPosition());
		telemetry.update();

		left.setPosition(gamepad1.left_trigger);
		right.setPosition(gamepad1.right_trigger);
	}
}
