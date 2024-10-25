package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.ServoStateMachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class ServoTest extends RobotPlugin {
	List<String> servoNames;
	OpMode opMode;
	ServoStateMachine stateMachine;
	GamepadManager gamepad;
	Telemetry telemetry;

	public ServoTest(OpMode opMode, String ...servoNames) {
		this.opMode = opMode;
		this.gamepad = new GamepadManager(opMode.gamepad1);
		this.telemetry = opMode.telemetry;
		this.servoNames = Arrays.asList(servoNames);
	}

	public void init() {
		ServoStateMachine.Builder builder = new ServoStateMachine.Builder();
		servoNames.forEach(name -> builder.addServo(name, s -> {}));
		builder.addState("none", new String[0], new float[0]);
		stateMachine = builder.build(opMode);
	}

	int servo = 0;
	Servo current;

	@Override
	public void start() {
		telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
		stateMachine.setCurrentState("none");
		current = stateMachine.getServo(servoNames.get(servo));
	}

	private void inc() {
		if (servo == servoNames.size() - 1) {
			servo = 0;
		} else {
			servo++;
		}
	}

	private void dec() {
		if (servo == 0) {
			servo = servoNames.size() - 1;
		} else {
			servo--;
		}
	}

	public void loop() {
		gamepad.poll();

		double incrementValue = gamepad.pressed(GamepadManager.Button.A) ? 0.01 : 0.1;

		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			inc();
			current = stateMachine.getServo(servoNames.get(servo));
		}

		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			dec();
			current = stateMachine.getServo(servoNames.get(servo));
		}

		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT)) {
			current.setPosition(current.getPosition() + incrementValue);
		}
		else if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			current.setPosition(current.getPosition() - incrementValue);
		}
		else if (gamepad.justPressed(GamepadManager.Button.B)) {
			current.setPosition(0.5);
		}


		telemetry.addData("Servos", fmt());
		telemetry.addData("Current Servo", servoNames.get(servo));
		telemetry.addData("Current Servo Position", current.getPosition());
		telemetry.update();
	}

	private String fmt() {
		return servoNames.stream()
			.map(n -> String.format(n.equals(servoNames.get(servo)) ? "(%s)" : " %s ", n))
			.collect(Collectors.joining(", "));
	}

}
