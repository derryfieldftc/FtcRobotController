package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PalmsOfGod {
	enum Position {
		Up,
		Down
	}
	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	public ColorSensor rightEye, leftEye;
	public Servo rightPalm, leftPalm;
	public double rightUp = .71, rightDown = 1;
	public double leftUp = .38, leftDown = 0;

	public PalmsOfGod(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	public void init() {
		rightPalm = hardwareMap.servo.get("rightPalm");
		leftPalm = hardwareMap.servo.get("leftPalm");

		rightEye = hardwareMap.colorSensor.get("rightEye");
		leftEye = hardwareMap.colorSensor.get("leftEye");
	}

	public PalmsOfGod setRightPalm(Position pos) {
		switch (pos) {
			case Up:
				rightPalm.setPosition(rightUp);
				break;
			case Down:
				rightPalm.setPosition(rightDown);
				break;
		}
		return this;
	}

	public PalmsOfGod setLeftPalm(Position pos) {
		switch (pos) {
			case Up:
				leftPalm.setPosition(leftUp);
				break;
			case Down:
				leftPalm.setPosition(leftDown);
				break;
		}
		return this;
	}

	//TODO! get color sensor working
	public Field.Ball getLeftBall() {
		return Field.Ball.None;
	}
	public Field.Ball getRightBall() {
		return Field.Ball.None;
	}
}
