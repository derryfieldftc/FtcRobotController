package org.firstinspires.ftc.teamcode.plugins.devices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;

public class ServoHandler implements DeviceHandler {
	OpMode opMode;
	Telemetry telemetry;
	public ServoHandler(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;

	}

	@Override
	public void info(DeviceTest.Part part) {
		Servo servo = (Servo) part.device;
		telemetry.addData("Position", servo.getPosition());
		telemetry.addData("Port", servo.getPortNumber());

	}

	@Override
	public DeviceTest.Part init(HardwareDevice device, String name, HardwareMap hardwareMap) {
		DeviceTest.Part part = new DeviceTest.Part();
		Servo servo = hardwareMap.servo.get(name);
		part.targetPos = servo.getPosition();
		servo.setPosition(part.targetPos);
		part.type = DeviceTest.Device.Servo;
		part.device = servo;
		telemetry.addLine("Servo: " + name);
		return part;
	}

	@Override
	public void editValues(DeviceTest.Part part, GamepadManager gamepad) {
		double scale = (gamepad.justPressed(GamepadManager.Button.DPAD_UP) || gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) ? .05 : 0;
		if (gamepad.pressed(GamepadManager.Button.DPAD_UP)) part.targetPos += scale;
		if (gamepad.pressed(GamepadManager.Button.DPAD_DOWN)) part.targetPos -= scale;

	}

	public void updateValues(DeviceTest.Part part) {
		Servo servo = (Servo) part.device;
		servo.setPosition(part.targetPos);
	}

	@Override
	public void helpMenu() {
		telemetry.addLine("^: raises target position by .05");
		telemetry.addLine("v: lowers target position by .05");

	}
}
