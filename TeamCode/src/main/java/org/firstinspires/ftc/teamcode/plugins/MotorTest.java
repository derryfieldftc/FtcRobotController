package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.util.ArrayList;

public class MotorTest extends RobotPlugin {
	ArrayList<String> motorNames = new ArrayList<>();
	Motor[] motors;
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	GamepadManager gamepad;

	public MotorTest(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		gamepad = new GamepadManager(opMode.gamepad1);
	}

	public MotorTest addMotor(String name) {
		motorNames.add(name);
		return this;
	}

	public void init() {
		motors = new Motor[motorNames.size()];
		for (int i = 0; i < motorNames.size(); i++) {
			motors[i] = new Motor();
			motors[i].motor = hardwareMap.dcMotor.get(motorNames.get(i));
			motors[i].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			motors[i].power = 0;
		}
	}

	int targetMotor = 0;
	public void loop() {
		gamepad.poll();

		if (gamepad.justPressed(GamepadManager.Button.A) && targetMotor < motors.length) {
			targetMotor++;
		} else if (gamepad.justPressed(GamepadManager.Button.Y) && targetMotor > 0) {
			targetMotor--;
		}
		if (gamepad.justPressed(GamepadManager.Button.B)) {
			motors[targetMotor].power += .05;
		}
		if (gamepad.justPressed(GamepadManager.Button.X) && motors[targetMotor].power > -1) {
			motors[targetMotor].power -= .05;
		}

		motors[targetMotor].motor.setPower(motors[targetMotor].power);

		for (int i = 0; i < motors.length; i++) {
			telemetry.addData((i == targetMotor) ? "=>" : "  " + motorNames.get(i), motors[i].motor.getPower());
			telemetry.addData("i:", i);
		}
		telemetry.update();


	}

	class Motor {
		public Motor() {
			power = 0.0;
			motor = null;
		}
		public double power;
		public DcMotor motor;
	}
}
