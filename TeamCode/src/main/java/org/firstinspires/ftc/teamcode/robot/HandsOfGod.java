package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class HandsOfGod extends RobotPart {
	public enum Position {
		Up,
		Down
	}

	private Gamepad gamepad;
	public Servo leftHand, rightHand;
	public boolean useGamepad = false, useTelemetry = false;
	private Position position;
	private double leftUp = .0;
	private double leftDown = 1;
	private double rightUp = 1;
	private double rightDown = 0;

	public HandsOfGod(OpMode opMode) {
		super(opMode);
		this.gamepad = opMode.gamepad2;
	}

	public void init() {
		leftHand = hardwareMap.servo.get("leftHand");
		rightHand = hardwareMap.servo.get("rightHand");
	}

	public HandsOfGod useTelemetry() {
		useTelemetry = true;
		return this;
	}

	public HandsOfGod useGamepad() {
		useGamepad = true;
		return this;
	}

	public void setPosition(Position pos) {
		this.position = pos;
		switch (pos) {
			case Up:
				leftHand.setPosition(leftUp);
				rightHand.setPosition(rightUp);
				break;
			case Down:
				leftHand.setPosition(leftDown);
				rightHand.setPosition(rightDown);

		}
	}

	public Position getPosition() {
		return this.position;
	}

	/**
	 * Only call if useTelemetry or useGamepad are true
	 */
	public void loop() {
		if (useGamepad) {
			leftHand.setPosition(gamepad.left_stick_y);
			rightHand.setPosition(gamepad.right_stick_y);
		}

		if (useTelemetry) {
			telemetry.addData("left", leftHand.getPosition());
			telemetry.addData("right", rightHand.getPosition());
		}
	}
}
