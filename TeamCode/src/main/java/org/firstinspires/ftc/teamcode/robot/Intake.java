package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	DcMotor intake;
	protected double speed = 0;

	public Intake(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	public void init() {
		intake = hardwareMap.dcMotor.get("intake");
	}

	public void setSpeed(double speed) {
		this.speed = speed;
	}

	public double getSpeed() {
		return speed;
	}

	public void loop() {
		intake.setPower(speed);
	}
}
