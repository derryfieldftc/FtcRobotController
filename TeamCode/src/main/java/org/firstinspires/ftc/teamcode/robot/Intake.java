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

	public Intake(OpMode opMode) {
		super(opMode);
		intake = hardwareMap.dcMotor.get(Part.Intake.name);
		intake.setDirection(DcMotorSimple.Direction.FORWARD);
	}

	public Intake setSpeed(double speed) {
		this.speed = speed;
		intake.setPower(speed);
		return this;
	}

	public double getSpeed() {
		return speed;
	}
}
