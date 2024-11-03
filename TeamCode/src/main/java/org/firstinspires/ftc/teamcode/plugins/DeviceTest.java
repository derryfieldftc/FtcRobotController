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
import org.firstinspires.ftc.teamcode.plugins.Devices.DcMotorHandler;
import org.firstinspires.ftc.teamcode.plugins.Devices.DeviceHandler;
import org.firstinspires.ftc.teamcode.plugins.Devices.ServoHandler;

import java.util.logging.Handler;

public class DeviceTest extends RobotPlugin {
	OpMode opMode;
	Telemetry telemetry;
	HardwareMap hardwareMap;
	String[] names;
	Part[] parts;
	GamepadManager gamepad;

	public enum Device {
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
				DeviceHandler handler;

				if (obj.getClass() == DcMotorImplEx.class) {
					handler = new DcMotorHandler( (LinearOpMode) opMode);
				} else if (obj.getClass() == ServoImplEx.class) {
					handler = new ServoHandler( (LinearOpMode) opMode);
				} else {
					throw new RuntimeException("Device " + names[i] + " is not found");
				}

				parts[i] = handler.init(obj, names[i], hardwareMap);
			} catch (Exception e) {
				throw  new RuntimeException("ERROR" + e);
			}
			telemetry.update();
		}
	}

	@Override
	public void start() {
		telemetry.clearAll();
	}

	int currentPartIndex = 0;
	@Override
	public void loop() {
		gamepad.poll();
		Part currentDevice = parts[currentPartIndex];
		if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT)) {
			currentPartIndex = (currentPartIndex == 0) ? parts.length - 1 : currentPartIndex - 1;
		}
		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT)) {
			currentPartIndex = (currentPartIndex == parts.length - 1) ? 0 : currentPartIndex + 1;
		}

		telemetry.clearAll();

		if (gamepad.pressed(GamepadManager.Button.START)) {
			showHelp(currentDevice);
		} else {
			telemetry.addLine(list());
			update(currentDevice);

			for (Part part : parts) {
				if (part == null) continue;
				if (part.targetPos == Double.NaN) continue;
				DeviceHandler handler = getDevice(part);
				handler.updateValues(part);
			}

		}
	}

	private void showHelp(Part currentDevice) {
		telemetry.addLine("General:");
		telemetry.addLine(">: increment current device");
		telemetry.addLine("<: decrement current device");
		telemetry.addLine("START: show help");
		telemetry.addLine();
		telemetry.addLine(currentDevice.type.toString() + ":");
		DeviceHandler handler = getDevice(currentDevice);
		handler.helpMenu();
	}

	private void update(Part currentDevice) {
		if (currentDevice == null) return;
		DeviceHandler handler = getDevice(currentDevice);
		handler.info(currentDevice);
		handler.editValues(currentDevice, gamepad);
	}

	private DeviceHandler getDevice(Part part) {
		switch (part.type) {
			case Motor:
				return new DcMotorHandler( (LinearOpMode) opMode);
			case Servo:
				return new ServoHandler( (LinearOpMode) opMode);
			default:
				throw new RuntimeException("Please add implementation for this device");
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

	public static class Part {
		public Device type;
		public HardwareDevice device;
		public double power;
		public double targetPos;
	}
}
