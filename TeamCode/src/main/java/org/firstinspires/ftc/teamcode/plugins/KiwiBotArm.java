package org.firstinspires.ftc.teamcode.plugins;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.lib.MotorManager;

public class KiwiBotArm extends RobotPlugin {
	OpMode opMode;
	Gamepad gamepad;
	MotorManager arm;
	int startingPos, targetPosition;
	DigitalChannel limitSwitch;

	public KiwiBotArm(OpMode opMode) {
		this.opMode = opMode;
		gamepad = opMode.gamepad2;
		MotorManager.setGlobalOpMode(opMode);
	}

	@Override
	public void init() {
		arm = new MotorManager("lift");
		limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "motorStop");
		arm._setTargetPosition(arm.getCurrentPosition())._setMode(DcMotor.RunMode.RUN_TO_POSITION)._setPower(.3);
	}

	@Override
	public void init_loop() {
		if (!limitSwitch.getState()) {
			startingPos = arm.getCurrentPosition();
			arm.setTargetPosition(arm.getCurrentPosition());
			arm.setUpperBound(startingPos);
			arm.setLowerBound(startingPos - 525);
			return;
		}

		arm.setTargetPosition(arm.getCurrentPosition() + 20);
		arm.dumpMotorData();

	}

	@Override
	public void start() {
		targetPosition = arm.getCurrentPosition();
	}

	@Override
	public void loop() {
		opMode.telemetry.addLine("" + limitSwitch.getState());
		arm.dumpMotorData();
		if (gamepad.a) {
			targetPosition = arm.getCurrentPosition();
		} else {
			arm.setTargetPosition(targetPosition);
		}

		if (gamepad.b) {
			targetPosition = startingPos - 150;
		}

		if (gamepad.a) {
			targetPosition = arm.getLowerBound();
		}

		if (gamepad.y) {
			targetPosition = arm.getUpperBound();
		}

		targetPosition = clamp(targetPosition - (int)(gamepad.left_stick_y * 3), arm.getLowerBound(), arm.getUpperBound());
	}
}
