package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
				} else if (obj.getClass() == DigitalChannel.class) {
					handler = new DigitalChannelHandler( (LinearOpMode) opMode);
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

	public static interface DeviceHandler {
		void info(Part part);
		Part init(HardwareDevice device, String name, HardwareMap hardwareMap);
		void editValues(Part part, GamepadManager gamepad);
		void updateValues(Part part);
		void helpMenu();

	}

	public static class DcMotorHandler implements DeviceHandler {
		OpMode opMode;
		Telemetry telemetry;
		public DcMotorHandler(LinearOpMode opMode) {
			this.opMode = opMode;
			this.telemetry = opMode.telemetry;

		}

		@Override
		public Part init(HardwareDevice device, String name, HardwareMap hardwareMap) {
			Part part = new Part();
			part.targetPos = 0;
			DcMotor motor = hardwareMap.dcMotor.get(name);
			motor.setTargetPosition((int)part.targetPos);
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			part.power = .3;
			part.device = motor;
			part.type = Device.Motor;
			telemetry.addLine("Motor: " + name);

			return part;
		}

		@Override
		public void info(Part part) {
			DcMotor motor = (DcMotor) part.device;
			telemetry.addData("Position", motor.getCurrentPosition());
			telemetry.addData("Target Position", motor.getTargetPosition());
			telemetry.addData("Power", motor.getPower());
			telemetry.addData("Port", motor.getPortNumber());
			telemetry.addData("Mode", motor.getMode());
			telemetry.addData("0 Power Behavior", motor.getZeroPowerBehavior());
		}

		@Override
		public void editValues(Part part, GamepadManager gamepad) {
			double scale;
			DcMotor motor = (DcMotor) part.device;
			if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
				scale = 5;
			} else {
				scale = .05;
			}
			if (gamepad.pressed(GamepadManager.Button.Y)) {
				DcMotor.RunMode prev = motor.getMode();
				motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				motor.setMode(prev);
			}
			if (gamepad.pressed(GamepadManager.Button.B)) {
				part.power = 0;
				part.targetPos = ( (DcMotor) part.device).getCurrentPosition();
			}
			if (gamepad.justPressed(GamepadManager.Button.X))
				motor.setZeroPowerBehavior((motor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
			if (gamepad.justPressed(GamepadManager.Button.A))
				motor.setMode((motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);

			if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
				if (gamepad.pressed(GamepadManager.Button.DPAD_UP)) part.targetPos += scale;
				if (gamepad.pressed(GamepadManager.Button.DPAD_DOWN)) part.targetPos -= scale;
			} else {
				if (gamepad.pressed(GamepadManager.Button.DPAD_UP)) part.power += (part.power < 1) ? scale : 0;
				if (gamepad.pressed(GamepadManager.Button.DPAD_DOWN)) part.power -= (part.power > 0) ? scale : 0;

			}
		}

		@Override
		public void updateValues(Part part) {
			DcMotor motor = (DcMotor) part.device;
			if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
				motor.setTargetPosition( (int) part.targetPos);
			else
				motor.setPower(part.power);
		}

		@Override
		public void helpMenu() {
			telemetry.addLine("A: toggles mode of motor");
			telemetry.addLine("X: toggles 0 power mode");
			telemetry.addLine("B: kills motor, must restart OpMode after this");
			telemetry.addLine("Y: resets the encoders of the current motor");
			telemetry.addLine("^: raises target position or power by 5 or .5");
			telemetry.addLine("v: lowers target position or power by 5 or .5");

		}
	}

	public static class ServoHandler implements DeviceHandler {
		OpMode opMode;
		Telemetry telemetry;
		public ServoHandler(LinearOpMode opMode) {
			this.opMode = opMode;
			this.telemetry = opMode.telemetry;

		}

		@Override
		public void info(Part part) {
			Servo servo = (Servo) part.device;
			telemetry.addData("Position", servo.getPosition());
			telemetry.addData("Port", servo.getPortNumber());

		}

		@Override
		public Part init(HardwareDevice device, String name, HardwareMap hardwareMap) {
			Part part = new Part();
			Servo servo = hardwareMap.servo.get(name);
			part.targetPos = servo.getPosition();
			servo.setPosition(part.targetPos);
			part.type = Device.Servo;
			part.device = servo;
			telemetry.addLine("Servo: " + name);
			return part;
		}

		@Override
		public void editValues(Part part, GamepadManager gamepad) {
			double scale = (gamepad.justPressed(GamepadManager.Button.DPAD_UP) || gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) ? .05 : 0;
			if (gamepad.pressed(GamepadManager.Button.DPAD_UP)) part.targetPos += scale;
			if (gamepad.pressed(GamepadManager.Button.DPAD_DOWN)) part.targetPos -= scale;

		}

		public void updateValues(Part part) {
			Servo servo = (Servo) part.device;
			servo.setPosition(part.targetPos);
		}

		@Override
		public void helpMenu() {
			telemetry.addLine("^: raises target position by .05");
			telemetry.addLine("v: lowers target position by .05");

		}
	}

	public class DigitalChannelHandler implements DeviceHandler {
		OpMode opMode;
		Telemetry telemetry;
		public DigitalChannelHandler(LinearOpMode opMode) {
			this.opMode = opMode;
			this.telemetry = opMode.telemetry;

		}

		@Override
		public void info(Part part) {
			DigitalChannel channel = (DigitalChannel) part.device;
			telemetry.addData("State", channel.getState());
		}

		@Override
		public Part init(HardwareDevice device, String name, HardwareMap hardwareMap) {
			Part part = new Part();
			DigitalChannel channel = hardwareMap.digitalChannel.get(name);
			part.device = channel;
			return part;
		}

		@Override
		public void editValues(Part part, GamepadManager gamepad) {

		}

		@Override
		public void updateValues(Part part) {

		}

		@Override
		public void helpMenu() {
			telemetry.addLine("Takes no input, just shows a switches state");
		}
	}
}
