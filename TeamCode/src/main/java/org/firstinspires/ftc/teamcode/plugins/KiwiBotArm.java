package org.firstinspires.ftc.teamcode.plugins;

import static androidx.core.math.MathUtils.clamp;

import static java.lang.Math.max;
import static java.lang.Math.min;

import android.content.SharedPreferences;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.lib.GamepadManager;
import org.firstinspires.ftc.teamcode.lib.MotorManager;

public class KiwiBotArm extends RobotPlugin {
	OpMode opMode;
	Gamepad gamepad;
	GamepadManager gamepadManager;
	MotorManager arm;
	Servo claw;
	int startingPos, targetPosition, floorDistance;

	DigitalChannel limitSwitch;

	public KiwiBotArm(OpMode opMode) {
		this.opMode = opMode;
		gamepad = opMode.gamepad1;
		MotorManager.setGlobalOpMode(opMode);
		gamepadManager = new GamepadManager(gamepad);
//		floorDistance = MediaStore.Files.getContentUri();

	}

	@Override
	public void init() {
		floorDistance = 320;
		arm = new MotorManager("lift");
		limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "motorStop");
		arm._setTargetPosition(arm.getCurrentPosition())._setMode(DcMotor.RunMode.RUN_TO_POSITION)._setPower(.3);
		claw = opMode.hardwareMap.servo.get("manipulator");
	}

	@Override
	public void init_loop() {
		if (!limitSwitch.getState()) {
			startingPos = arm.getCurrentPosition();
			arm.setTargetPosition(arm.getCurrentPosition());
			arm.setUpperBound(startingPos);
			arm.setLowerBound(startingPos - floorDistance);
			return;
		}

		arm.setTargetPosition(arm.getCurrentPosition() + 20);
		arm.dumpMotorData();

	}

	@Override
	public void start() {
		targetPosition = arm.getCurrentPosition();
	}

	boolean clawOpen = false;
	double clawOpenPos = .31, clawClosedPos = .5;

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

		if (gamepad.dpad_up) {
			arm.setLowerBound(arm.getLowerBound() + 1);
		}

		if (gamepad.dpad_down) {
			arm.setLowerBound(arm.getLowerBound() - 1);
		}

		targetPosition = clamp(targetPosition - (int)(gamepad.right_stick_y * 3), arm.getLowerBound(), arm.getUpperBound());

		if (gamepadManager.justPressed(GamepadManager.Button.X)) {
			clawOpen = !clawOpen;
		}

		if (clawOpen) {
			claw.setPosition(clawOpenPos);
		} else {
			claw.setPosition(clawClosedPos);
		}
		opMode.telemetry.addData("Bounds U", arm.getUpperBound() + " L: " + arm.getLowerBound());
		opMode.telemetry.addData("Arm position", arm.getCurrentPosition());
		opMode.telemetry.addData("claw", gamepad.right_trigger);
		gamepadManager.poll();
	}
}
