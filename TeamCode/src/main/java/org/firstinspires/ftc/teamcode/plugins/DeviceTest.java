package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class DeviceTest extends RobotPlugin {
	OpMode opMode;
	Telemetry telemetry;
	HardwareMap hardwareMap;
	String[] names;
	Part[] parts;
	GamepadManager gamepad;
	int enabled = 0;
	boolean errored = false;

	enum Device {
		Servo,
		Motor,
		//add more when necessary
	}

	public DeviceTest(LinearOpMode opMode, String... names) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
		this.names = names;
		this.gamepad = new GamepadManager(opMode.gamepad2);
		parts = new Part[names.length];
	}

	@Override
	public void init() {
		telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
		for (int i = 0; i < parts.length; i++) {
			try {
				HardwareDevice obj = hardwareMap.get(names[i]);
				Part part = new Part();

				if (obj.getClass() == DcMotorImplEx.class) {
					part.targetPos = 0;
					DcMotor motor = hardwareMap.dcMotor.get(names[i]);
					motor.setTargetPosition((int)part.targetPos);
					motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					part.power = .3;
					part.device = motor;
					part.type = Device.Motor;
					telemetry.addLine("Motor: " + names[i]);
				} else if (obj.getClass() == ServoImplEx.class) {
					Servo servo = hardwareMap.servo.get(names[i]);
					part.targetPos = servo.getPosition();
					servo.setPosition(part.targetPos);
					part.type = Device.Servo;
					part.device = servo;
					telemetry.addLine("Servo: " + names[i]);
				} else {
					telemetry.addData("somthings messed up", obj.getDeviceName());
					errored = true;
				}

				parts[i] = part;
			} catch (Exception e) {
				telemetry.addData("ERROR", e);
			}
			telemetry.update();
		}
	}

	@Override
	public void start() {
		if (!errored) telemetry.clearAll();
	}

	int currentPartIndex = 0;
	@Override
	public void loop() {
		gamepad.poll();
		Part currentDevice = parts[currentPartIndex];
		if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			currentPartIndex = (currentPartIndex == 0) ? parts.length - 1 : currentPartIndex - 1;
			telemetry.clearAll();
		}
		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT)) {
			currentPartIndex = (currentPartIndex == parts.length - 1) ? 0 : currentPartIndex + 1;
			telemetry.clearAll();
		}

		update(currentDevice);

		for (Part part: parts) {
			if (part == null) continue;
			if (part.targetPos == Double.NaN) continue;
			switch (part.type) {
				case Motor:
					DcMotor motor = (DcMotor) part.device;
					motor.setTargetPosition( (int) part.targetPos);
					motor.setPower(part.power);
					break;
				case Servo:
					Servo servo = (Servo) part.device;
					servo.setPosition(part.targetPos);
					break;
				default:
					break;
			}
		}

		telemetry.addLine(list());
		dumpInfo(currentDevice);
	}

	private void update(Part currentDevice) {
		if (currentDevice == null) return;
		double scale = 0;
		switch (currentDevice.type) {
			case Motor:
				DcMotor motor = (DcMotor) currentDevice.device;
				if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
					scale = 5;
				} else {
					scale = .05;
				}
				if (gamepad.pressed(GamepadManager.Button.B)) {
					DcMotor.RunMode prev = motor.getMode();
					motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					motor.setMode(prev);
				}
				if (gamepad.justPressed(GamepadManager.Button.X)) currentDevice.power *= -1;
				if (gamepad.justPressed(GamepadManager.Button.A))
					motor.setMode((motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
				break;
			case Servo:
				scale = (gamepad.justPressed(GamepadManager.Button.DPAD_UP) || gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) ? .05 : 0;
				break;
		}
		if (gamepad.pressed(GamepadManager.Button.DPAD_UP)) currentDevice.targetPos += scale;
		if (gamepad.pressed(GamepadManager.Button.DPAD_DOWN)) currentDevice.targetPos -= scale;
	}

	private void dumpInfo(Part currentDevice) {
		if (currentDevice == null) return;
		switch (currentDevice.type) {
			case Motor:
				DcMotor motor = (DcMotor) currentDevice.device;
				telemetry.addData("Position", motor.getCurrentPosition());
				telemetry.addData("Target Position", motor.getTargetPosition());
				telemetry.addData("Power", motor.getPower());
				telemetry.addData("Port", motor.getPortNumber());
				telemetry.addData("Mode", motor.getMode());
				break;
			case Servo:
				Servo servo = (Servo) currentDevice.device;
				telemetry.addData("Position", servo.getPosition());
				telemetry.addData("Port", servo.getPortNumber());
				break;
			default:
				telemetry.addLine("Please add implementation for this device");
		}
	}

	private String list() {
		StringBuilder stringBuilder = new StringBuilder();
		for (int i = 0; i < parts.length; i++) {
			stringBuilder.append((i == currentPartIndex) ? "(" + names[i] + ")" : names[i]);
			stringBuilder.append((i == parts.length - 1) ? "" : ", ");
		}
		return stringBuilder.toString();
	}

	private class Part {
		Device type;
		HardwareDevice device;
		double power;
		double targetPos;
	}
}
