package org.firstinspires.ftc.teamcode.plugins.Devices;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;

public class DcMotorHandler implements DeviceHandler {
	OpMode opMode;
	Telemetry telemetry;
	public DcMotorHandler(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;

	}

	@Override
	public DeviceTest.Part init(HardwareDevice device, String name, HardwareMap hardwareMap) {
		DeviceTest.Part part = new DeviceTest.Part();
		part.targetPos = 0;
		DcMotor motor = hardwareMap.dcMotor.get(name);
		motor.setTargetPosition((int)part.targetPos);
		motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		part.power = .3;
		part.device = motor;
		part.type = DeviceTest.Device.Motor;
		telemetry.addLine("Motor: " + name);

		return part;
	}

	@Override
	public void info(DeviceTest.Part part) {
		DcMotor motor = (DcMotor) part.device;
		telemetry.addData("Position", motor.getCurrentPosition());
		telemetry.addData("Target Position", motor.getTargetPosition());
		telemetry.addData("Power", motor.getPower());
		telemetry.addData("Port", motor.getPortNumber());
		telemetry.addData("Mode", motor.getMode());
		telemetry.addData("0 Power Behavior", motor.getZeroPowerBehavior());
	}

	@Override
	public void editValues(DeviceTest.Part part, GamepadManager gamepad) {
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
	public void updateValues(DeviceTest.Part part) {
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
