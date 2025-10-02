package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	DcMotor intake;
	protected double speed = 0;
	ColorSensor colorSensor;

	public Intake(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	public void init() {
		intake = hardwareMap.dcMotor.get("intake");
		intake.setDirection(DcMotorSimple.Direction.FORWARD);
		colorSensor = hardwareMap.colorSensor.get("intakeColorSensor");
	}

	public Field.Ball getBallType() {
		float[] temp = {0f, 0f, 0f};
		Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), temp);
		return Field.Ball.getBallFromColor(temp);
	}

	public void setSpeed(double speed) {
		this.speed = speed;
		intake.setPower(speed);
	}

	public double getSpeed() {
		return speed;
	}

	public void loop() {
		intake.setPower(speed);
	}

	public Action enable() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				setSpeed(1);
				return false;
			}
		};
	}

	public Action disable() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				setSpeed(0);
				return false;
			}
		};
	}

	public Action RR_setSpeed(double speed) {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				Intake.this.setSpeed(speed);
				return false;
			}
		};
	}
}
