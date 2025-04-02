package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.lib.MotorManager;

public class MotorManagerTest extends RobotPlugin {
	OpMode opMode;
	MotorManager motor;
	Gamepad gamepad;
	int targetPos = 0;

	public MotorManagerTest(OpMode opMode) {
		this.opMode = opMode;
		MotorManager.setGlobalOpMode(opMode);
		gamepad = opMode.gamepad2;
	}

	public void init() {
		motor = new MotorManager("test")._setMode(DcMotor.RunMode.RUN_USING_ENCODER).setPowerLimit(.7f);
	}

	public void loop() {
		motor.setPower(gamepad.left_stick_y);
		motor.dumpMotorData();

		if (gamepad.a) {
			motor.setPowerLimit(gamepad.right_stick_y);
		}

		if (gamepad.b) {
			motor._setTargetPosition(targetPos).setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}

		if (gamepad.y) {
			motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		if (gamepad.x) {
			targetPos = (int) (gamepad.left_stick_x * 10000);
		}

		if (gamepad.right_stick_button) {
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
		if (gamepad.right_bumper) {
			motor.addBounds(0, (int)(gamepad.right_stick_x * 10000));
		}
		motor._setTargetPosition(targetPos);
	}
}
