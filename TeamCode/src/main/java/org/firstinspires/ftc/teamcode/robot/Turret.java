package org.firstinspires.ftc.teamcode.robot;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//Oh boy
public class Turret {
	DcMotor rotator; //25 to 95 ratio, 1 full rotation is 2k steps
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	Gamepad gamepad;
	TouchSensor limit;
	int maxDelta = 2000;
	double rotatorPower = 0;
	boolean useGamePad;
	boolean useCamera;
	PID rotation;

	public Turret(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		gamepad = opMode.gamepad2;
	}

	public Turret useCamera() {
		useCamera = true;
		return this;
	}

	public Turret useGamepad() {
		useGamePad = true;
		return this;
	}

	public void init() {
		rotator = hardwareMap.dcMotor.get("turretRotator");
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		limit = hardwareMap.touchSensor.get("turretLimit");
		//TODO! Fix this, all of this
		while (!limit.isPressed()) {
			rotator.setPower(.1);
		}
		rotator.setPower(0);
		rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//TODO! Test values
		rotation = new PID(.33, .33, .33, .01);
	}

	public void setRotatorPower(double power) {
		rotatorPower = power;
	}

	public void loop() {
		if (useGamePad) {
			rotator.setPower(gamepad.right_stick_y);
			if (gamepad.y) {
				rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
		}

		if (abs(rotator.getCurrentPosition()) >= maxDelta) {
			return;
		}
		rotator.setPower(rotatorPower);

		telemetry.addData("motorpos", rotator.getCurrentPosition());
		telemetry.addData("limit", limit.getValue());
		telemetry.update();
	}

	// TODO how many encode ticks to do a full rotation
	public double getRotation() {
		return 0;
	}

}
