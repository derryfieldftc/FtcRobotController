package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

import java.util.ArrayList;

public class ServoTest extends RobotPlugin {
	private Servo[] servos;
	private ArrayList<String> servoNames = new ArrayList<>();
	OpMode opMode;
	HardwareMap hardwareMap;
	Gamepad gamepad;
	Telemetry telemetry;

	public ServoTest(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad = opMode.gamepad1;
		this.telemetry = opMode.telemetry;
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

	boolean dpadPressed = false;
	int targetServo = 0;
	double targetPos = 0;

	public void loop() {
		if (gamepad.dpad_left && !dpadPressed) {
			if (targetServo > 0) {
				targetServo--;
			}
			dpadPressed = true;
		} else if (gamepad.dpad_right && !dpadPressed) {
			if (targetServo < servos.length - 1) {
				targetServo++;
			}
			dpadPressed = true;
		}
		if (gamepad.a) {
			dpadPressed = false;
		}

		if (gamepad.right_bumper) {
			targetServo = servos.length - 1;
		} else if (gamepad.left_bumper) {
			targetServo = 0;
		}

		if (gamepad.dpad_up) {
			targetPos += .005;
		} else if (gamepad.dpad_down) {
			targetPos -= .005;
		}
		servos[targetServo].setPosition(targetPos);

		for (int i = 0; i < servos.length; i++) {
			Servo servo = servos[i];
			telemetry.addData((targetServo == i ? "=>" : "  ") + servo.getDeviceName(), servo.getPosition());
		}
		telemetry.update();
	}
}
