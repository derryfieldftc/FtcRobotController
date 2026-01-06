package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotPart;

@TeleOp
public class LiftHeightTest extends OpMode {
	Servo lift;

	@Override
	public void init() {
		lift = hardwareMap.servo.get(RobotPart.Part.LiftServo.name);

	}

	@Override
	public void loop() {
		lift.setPosition(gamepad1.left_stick_y);

		telemetry.addData("pos", lift.getPosition());
		telemetry.update();
	}
}
