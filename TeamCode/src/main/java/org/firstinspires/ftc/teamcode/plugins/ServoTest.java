package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.util.ArrayList;

public class ServoTest extends RobotPlugin {
	private Servo[] servos;
	private ArrayList<String> servoNames = new ArrayList<>();
	OpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	GamepadManager gamepad;

	public ServoTest(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		gamepad = new GamepadManager(opMode.gamepad1);
	}

	public ServoTest addServo(String name) {
		servoNames.add(name);
		return this;
	}

	public void init() {
		servos = new Servo[servoNames.size()];
		for (int i = 0; i < servoNames.size(); i++) {
			servos[i] = hardwareMap.servo.get(servoNames.get(i));
		}
	}

	int targetServo = 0;
	double targetPos = 0;

	public void loop() {
		gamepad.poll();
		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) {
			if (targetServo > 0) {
				targetServo--;
			}
		} else if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			if (targetServo < servos.length - 1) {
				targetServo++;
			}
		}

		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) {
			targetServo = servos.length - 1;
		} else if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) {
			targetServo = 0;
		}

		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT) && targetPos < 1) {
			targetPos += .05;
		} else if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT) && targetPos > 0) {
			targetPos -= .05;
		}
		servos[targetServo].setPosition(targetPos);

		for (int i = 0; i < servos.length; i++) {
			Servo servo = servos[i];
			telemetry.addData((targetServo == i ? "=>" : "  ") + servoNames.get(i), servo.getPosition());
		}
		telemetry.update();
	}
}
