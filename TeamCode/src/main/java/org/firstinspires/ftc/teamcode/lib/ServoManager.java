package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class ServoManager implements Servo {
	public double min = 0, max = 1;
	Servo servo;
	static OpMode opMode;

	/**
	 * Creates a new ServoManager.
	 * If using this method please make sure <code>ServoManager.setGlobalOpMode()</code> has been run first
	 * @param name
	 * @see ServoManager#setGlobalOpMode(OpMode)
	 */
	public ServoManager(String name) {
		if (opMode == null)
			throw new RuntimeException("OpMode is null, please make sure you have used the ServoManager.setGlobalOpMode() method previously");

		this.servo = opMode.hardwareMap.servo.get(name);
	}

	/**
	 * Prefer the <code>ServoManager(String name)</code> constructor unless you need a reference to the servo
	 * @param servo
	 */
	public ServoManager(Servo servo) {
		this.servo = servo;
	}
	@Override
	public ServoController getController() {
		return servo.getController();
	}

	@Override
	public int getPortNumber() {
		return servo.getPortNumber();
	}

	public ServoManager _setDirection(Direction direction) {
		servo.setDirection(direction);
		return this;
	}

	@Override
	public void setDirection(Direction direction) {
		servo.setDirection(direction);
	}

	@Override
	public Direction getDirection() {
		return servo.getDirection();
	}

	public ServoManager _setPosition(double position) {
		servo.setPosition(position);
		return this;
	}

	@Override
	public void setPosition(double position) {
		servo.setPosition(position);
	}

	@Override
	public double getPosition() {
		return servo.getPosition();
	}

	/**
	 * Scales the servos movement range (basically acts as bounds)
	 * @param min
	 * @param max
	 * @return
	 */
	public ServoManager _scaleRange(double min, double max) {
		servo.scaleRange(min, max);
		this.min = min;
		this.max = max;
		return this;
	}

	@Override
	public void scaleRange(double min, double max) {
		servo.scaleRange(min, max);
		this.min = min;
		this.max = max;
	}

	@Override
	public Manufacturer getManufacturer() {
		return servo.getManufacturer();
	}

	@Override
	public String getDeviceName() {
		return servo.getDeviceName();
	}

	@Override
	public String getConnectionInfo() {
		return servo.getConnectionInfo();
	}

	@Override
	public int getVersion() {
		return servo.getVersion();
	}

	public ServoManager _resetDeviceConfigurationForOpMode() {
		servo.resetDeviceConfigurationForOpMode();
		return this;
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
		servo.resetDeviceConfigurationForOpMode();
	}

	@Override
	public void close() {
		servo.close();
	}

	public static void setGlobalOpMode(OpMode opMode) {
		ServoManager.opMode = opMode;
	}
}
