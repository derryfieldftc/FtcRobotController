package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class MotorHandler implements DcMotor {
	private DcMotor motor;

	int max = Integer.MAX_VALUE;
	int min = Integer.MIN_VALUE;

	public MotorHandler(DcMotor motor) {
		super();
		this.motor = motor;
	}

	public MotorHandler addBounds(int max, int min) {
		this.min = min;
		this.max = max;
		return this;
	}

	public MotorHandler addUpperBound(int max) {
		return this.addBounds(max, min);
	}

	public MotorHandler addLowerBound(int min) {
		return this.addBounds(max, min);
	}

	public void resetBounds() {
		max = Integer.MAX_VALUE;
		min = Integer.MIN_VALUE;
	}

	public MotorConfigurationType getMotorType() {
		return motor.getMotorType();
	}

	public void setMotorType(MotorConfigurationType motorType) {
		motor.setMotorType(motorType);
	}

	public DcMotorController getController() {
		return motor.getController();
	}

	public int getPortNumber() {
		return motor.getPortNumber();
	}

	public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
		motor.setZeroPowerBehavior(zeroPowerBehavior);
	}

	public ZeroPowerBehavior getZeroPowerBehavior() {
		return motor.getZeroPowerBehavior();
	}

	public void setPowerFloat() {
		motor.setPowerFloat();

	}

	public boolean getPowerFloat() {
		return motor.getPowerFloat();
	}

	public void setTargetPosition(int position) {
		motor.setTargetPosition((position <= max) ? (position >= min) ? position : min : max);

	}

	public int getTargetPosition() {
		return motor.getTargetPosition();
	}

	public boolean isBusy() {
		return motor.isBusy();
	}

	public int getCurrentPosition() {
		return motor.getCurrentPosition();
	}

	public void setMode(RunMode mode) {
		motor.setMode(mode);
	}

	public RunMode getMode() {
		return motor.getMode();
	}

	public void setDirection(Direction direction) {
		motor.setDirection(direction);
	}

	public Direction getDirection() {
		return motor.getDirection();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public double getPower() {
		return motor.getPower();
	}

	public Manufacturer getManufacturer() {
		return motor.getManufacturer();
	}

	public String getDeviceName() {
		return motor.getDeviceName();
	}

	public String getConnectionInfo() {
		return motor.getConnectionInfo();
	}

	public int getVersion() {
		return 0;
	}

	public void resetDeviceConfigurationForOpMode() {
		motor.resetDeviceConfigurationForOpMode();
		this.resetBounds();
	}

	public void close() {
		motor.close();
	}
}
