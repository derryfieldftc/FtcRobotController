package org.firstinspires.ftc.teamcode.lib;

import static androidx.core.math.MathUtils.clamp;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;

/**
 * Thin wrapper class that implements DcMotor.
 * Comes with a few quality of life features like speed limits and bounds.
 * Any methods that are prefixed with an `_` return this object
 * @see DcMotor
 */
public class MotorManager implements DcMotor {
	private int min = Integer.MIN_VALUE;
	private int max = Integer.MAX_VALUE;
	private float powerLimit = 1;
	private DcMotor motor;
	private static OpMode opMode;
	private static ArrayList<MotorManager> motorManagerList = new ArrayList<>();

	/**
	 * Creates a new MotorManager.
	 * If using this method please make sure <code>MotorManager.setGlobalOpMode()</code> has been run first
	 * @param name
	 * @see MotorManager#setGlobalOpMode(OpMode) 
	 */
	public MotorManager(String name) {
		if (opMode == null)
			throw new RuntimeException("OpMode is null, please make sure you have used the MotorManager.setGlobalOpMode() method");

		this.motor = opMode.hardwareMap.dcMotor.get(name);
		motorManagerList.add(this);
	}

	/**
	 * Prefer the <code>MotorManager(String name)</code> constructor unless you need a reference to the motor
	 * @param motor
	 */
	public MotorManager(DcMotor motor) {
		this.motor = motor;
		motorManagerList.add(this);
	}

	public MotorManager addBounds(int min, int max) {
		this.min = min;
		this.max = max;
		return this;
	}

	public MotorManager resetBounds() {
		this.min = Integer.MIN_VALUE;
		this.max = Integer.MAX_VALUE;
		return this;
	}

	public MotorManager setUpperBound(int max) {
		this.max = max;
		return this;
	}

	public int getUpperBound() {
		return max;
	}

	public MotorManager setLowerBound(int min) {
		this.min = min;
		return this;
	}

	public int getLowerBound() {
		return min;
	}

	/**
	 * Sets a power limit on the motor
	 * @param powerLimit not negative
	 * @return
	 */
	public MotorManager setPowerLimit(float powerLimit) {
		this.powerLimit = abs(powerLimit);
		return this;
	}

	public float getPowerLimit() {
		return powerLimit;
	}

	public MotorManager resetPowerLimit() {
		powerLimit = 1;
		return this;
	}

	@Override
	public void setPower(double power) {
		motor.setPower((abs(power) <= powerLimit) ? power : (power >= 0) ? powerLimit : -powerLimit);
	}

	/**
	 * {@link MotorManager#setPower}
	 * @Returns this
	 */
	public MotorManager _setPower(double power) {
		motor.setPower(clamp(power, min, max));
		return this;
	}

	/**
	 * Returns some data about a motor, useful for debugging.
	 * Changes what data is printed based on motor mode.
	 * @return
	 */
	public String motorData() {
		switch (getMode()) {
			case RUN_USING_ENCODER:
			case RUN_WITHOUT_ENCODER:
				return getMode()
						+ "\npower val/max: " + getPower()
						+ "/" + powerLimit
						+ "\ndir: " + getDirection();
			case RUN_TO_POSITION:
				return getMode()
						+ "\npos val/min/max: " + getCurrentPosition()
						+ "/" + min + "/" + max
						+ "\ntargetPos: " + getTargetPosition()
						+ "\npower val/max: " + getPower()
						+ "/" + powerLimit;
			default:
				return getMode()
						+ "\n" + getMotorType().toString()
						+ "\nport: " + getPortNumber()
						+ "\nname: " + getDeviceName()
						+ "\ncontroller: " + getController()
						+ "\nconnectionInfo: " + getConnectionInfo()
						+ "\nmanufacturer: " + getManufacturer();
		}
	}

	/**
	 * Automatically dumps <code>motorData()</code> to telemetry
	 */
	public void dumpMotorData() {
		opMode.telemetry.addLine(motorData());
	}

	@Override
	public void setDirection(Direction direction) {
		motor.setDirection(direction);
	}

	/**
	 * {@link MotorManager#setDirection}
	 * @Returns this
	 */
	public MotorManager _setDirection(Direction direction) {
		motor.setDirection(direction);
		return this;
	}

	@Override
	public Direction getDirection() {
		return motor.getDirection();
	}

	@Override
	public double getPower() {
		return motor.getPower();
	}

	@Override
	public MotorConfigurationType getMotorType() {
		return motor.getMotorType();
	}

	@Override
	public void setMotorType(MotorConfigurationType motorType) {
		motor.setMotorType(motorType);
	}

	/**
	 * {@link MotorManager#setMotorType}
	 * @Returns this
	 */
	public MotorManager _setMotorType(MotorConfigurationType motorType) {
		motor.setMotorType(motorType);
		return this;
	}

	@Override
	public DcMotorController getController() {
		return motor.getController();
	}

	@Override
	public int getPortNumber() {
		return motor.getPortNumber();
	}

	@Override
	public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
		motor.setZeroPowerBehavior(zeroPowerBehavior);
	}

	/**
	 * {@link MotorManager#setZeroPowerBehavior}
	 * @Returns this
	 */
	public MotorManager _setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
		motor.setZeroPowerBehavior(zeroPowerBehavior);
		return this;
	}

	@Override
	public ZeroPowerBehavior getZeroPowerBehavior() {
		return motor.getZeroPowerBehavior();
	}

	@Override
	public void setPowerFloat() {
		setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
		setPower(0);
	}

	/**
	 * {@link MotorManager#setPowerFloat}
	 * @Returns this
	 */
	public MotorManager _setPowerFloat() {
		setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
		setPower(0);
		return this;
	}

	@Override
	public boolean getPowerFloat() {
		return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0;
	}

	@Override
	public void setTargetPosition(int position) {
		motor.setTargetPosition(clamp(position, min, max));
	}

	/**
	 * {@link MotorManager#setTargetPosition}
	 * @Returns this
	 */
	public MotorManager _setTargetPosition(int position) {
		motor.setTargetPosition(clamp(position, min, max));
		return this;
	}

	@Override
	public int getTargetPosition() {
		return motor.getTargetPosition();
	}

	@Override
	public boolean isBusy() {
		return motor.isBusy();
	}

	@Override
	public int getCurrentPosition() {
		return motor.getCurrentPosition();
	}

	@Override
	public void setMode(RunMode mode) {
		motor.setMode(mode);
	}

	/**
	 * {@link MotorManager#setMode}
	 * @Returns this
	 */
	public MotorManager _setMode(RunMode mode) {
		motor.setMode(mode);
		return this;
	}

	@Override
	public RunMode getMode() {
		return motor.getMode();
	}

	@Override
	public Manufacturer getManufacturer() {
		return motor.getManufacturer();
	}

	@Override
	public String getDeviceName() {
		return motor.getDeviceName();
	}

	@Override
	public String getConnectionInfo() {
		return motor.getConnectionInfo();
	}

	@Override
	public int getVersion() {
		return motor.getVersion();
	}

	@Override
	public void resetDeviceConfigurationForOpMode() {
		motor.resetDeviceConfigurationForOpMode(); //TODO! figure out what this does
	}

	/**
	 * {@link MotorManager#resetDeviceConfigurationForOpMode}
	 * @Returns this
	 */
	public MotorManager _resetDeviceConfigurationForOpMode() {
		motor.resetDeviceConfigurationForOpMode(); //TODO! figure out what this does
		return this;
	}

	@Override
	public void close() {
		motor.close();
	}

	/**
	 * {@link MotorManager#close}
	 * @Returns this
	 */
	public MotorManager _close() {
		motor.close();
		return this;
	}

	/**
	 * Static method used for some other methods.
	 * Please include <code>MotorManager.setGlobalOpMode(this)</code> before instantiating any motors
	 * @param opMode
	 */
	public static void setGlobalOpMode(OpMode opMode) { MotorManager.opMode = opMode; }
	public static void cleanUpMotors() {} //TODO! write this
}
