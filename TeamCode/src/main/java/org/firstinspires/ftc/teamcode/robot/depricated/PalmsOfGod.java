package org.firstinspires.ftc.teamcode.robot.depricated;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.RobotPart;

@Deprecated
public class PalmsOfGod extends RobotPart {
	public enum Position {
		Up,
		Down
	}

	public ColorSensor rightEye, leftEye;
	public Servo rightPalm, leftPalm;
	public double rightUp = .55, rightDown = .25;
	public double leftUp = .23, leftDown = .45;
	public boolean useTelemetry = false;
	public Position leftPalmPos;
	public Position rightPalmPos;

	public enum Palm {
		Right,
		Left
	}

	public PalmsOfGod(OpMode opMode) {
		super(opMode);
	}

	public PalmsOfGod useTelemetry() {
		useTelemetry = true;
		return this;
	}

	public void init() {
		rightPalm = hardwareMap.servo.get(Part.RightPalm.name);
		leftPalm = hardwareMap.servo.get(Part.LeftPalm.name);

		rightEye = hardwareMap.colorSensor.get(Part.RightEye.name);
		leftEye = hardwareMap.colorSensor.get(Part.LeftEye.name);
	}

	public PalmsOfGod setRightPalm(Position pos) {
		rightPalmPos = pos;
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
		leftPalmPos = pos;
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

	public Field.Ball getLeftBall() {
		if (useTelemetry)
			telemetry.addLine(String.format("left: r: %d, g: %d, b: %d", leftEye.red(), leftEye.green(), leftEye.blue()));
		float[] temp = {0f, 0f, 0f};
		Color.RGBToHSV(leftEye.red(), leftEye.green(), leftEye.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.Left());
	}

	public Field.Ball getRightBall() {
		if (useTelemetry)
			telemetry.addLine(String.format("right: r: %d, g: %d, b: %d", rightEye.red(), rightEye.green(), rightEye.blue()));
		float[] temp = {0f, 0f, 0f};
		Color.RGBToHSV(rightEye.red(), rightEye.green(), rightEye.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.Right());
	}

	public Position getPalm(Palm palm) {
		Position ret;

		if (palm == Palm.Right) {
			return rightPalmPos;
		}
		return leftPalmPos;
	}
}
