package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HandsOfGod {
	public enum Position {
		Up,
		Down
	}

	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	private Gamepad gamepad;
	public Servo leftHand, rightHand;
	public boolean useGamepad = false, useTelemetry = false;
	private double leftUp = .65;
	private double leftDown = 1;
	private double rightUp = .38;
	private double rightDown = 0;

	public HandsOfGod(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
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

	/**
	 * Class for all RoadRunner Actions
	 */
	class RR {
		public Action setPositionUp() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket telemetryPacket) {
					setPosition(Position.Up);
					return false;
				}
			};
		}

		public Action setPositionDown() {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket telemetryPacket) {
					setPosition(Position.Down);
					return false;
				}
			};
		}

		public Action setPosition(Position pos) {
			return new Action() {
				@Override
				public boolean run(@NonNull TelemetryPacket telemetryPacket) {
					HandsOfGod.this.setPosition(pos);
					return false;
				}
			};
		}
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
