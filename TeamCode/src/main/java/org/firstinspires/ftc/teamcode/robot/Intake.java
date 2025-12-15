package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends RobotPart {
	DcMotor intake;
	protected double speed = 0;
	ColorSensor colorSensor;
	Servo intakeLift;
	protected double height = 0;

	public Intake(OpMode opMode) {
		super(opMode);
	}

	public void init() {
		intake = hardwareMap.dcMotor.get(Part.Intake.name);
		intake.setDirection(DcMotorSimple.Direction.FORWARD);
		colorSensor = hardwareMap.colorSensor.get("intakeColorSensor");
		intakeLift = hardwareMap.servo.get(Part.IntakeLift.name);
	}

	public Field.Ball getIntakeBall() {
		float[] temp = {0f, 0f, 0f};
		Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), temp);
		return Field.Ball.getBallFromColor(temp, new Field.ColorSensorValues.Intake());
	}

	public Intake setSpeed(double speed) {
		this.speed = speed;
		intake.setPower(speed);
		return this;
	}

	public Intake setHeight(double height) {
		this.height = height;
		return this;
	}

	public double getSpeed() {
		return speed;
	}

	public void loop() {
		intake.setPower(speed);
		intakeLift.setPosition(height);
	}
}
