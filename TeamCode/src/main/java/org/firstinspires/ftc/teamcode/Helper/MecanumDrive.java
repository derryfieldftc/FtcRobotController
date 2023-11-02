package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.DriveMotorConfig;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.stream.DoubleStream;

public class MecanumDrive {

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;

	public MecanumDrive(
			DcMotor rightFront,
			DcMotor leftFront,
			DcMotor rightRear,
			DcMotor leftRear
	) {
		this.rightFront = rightFront;
		this.leftFront = leftFront;
		this.rightRear = rightRear;
		this.leftRear = leftRear;
		configureMotors();
	}

	public MecanumDrive(
			HardwareMap hardwareMap,
			String rightFrontMotorName,
			String leftFrontMotorName,
			String rightRearMotorName,
			String leftRearMotorName
	) {
		rightFront = (DcMotor)hardwareMap.get(rightFrontMotorName);
		leftFront = (DcMotor)hardwareMap.get(leftFrontMotorName);
		rightRear = (DcMotor)hardwareMap.get(rightRearMotorName);
		leftRear = (DcMotor)hardwareMap.get(leftRearMotorName);
		configureMotors();
	}

	private void configureMotors() {
		rightFront.setDirection(DcMotor.Direction.FORWARD);
		leftFront.setDirection(DcMotor.Direction.REVERSE);
		rightRear.setDirection(DcMotor.Direction.FORWARD);
		leftRear.setDirection(DcMotor.Direction.REVERSE);

		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void drive(double forward, double strafe, double rotate, double scale) {
		calculateMotorPower(forward, strafe, rotate)
				.map(power -> power * scale)
				.applyPowerTo(rightFront, leftFront, rightRear, leftRear);
	}

	public void drive(double forward, double strafe, double rotate) {
		drive(forward, strafe, rotate, 1.0);
	}

	/**
	 * British spelling because fuck you
	 */
	public void driveCentimetresForward(double centimetres, double power) {

	}

	/**
	 * British spelling because fuck you
	 */
	public void driveCentimetresStrafe(double centimetres, double power) {

	}

	private static DriveMotorConfig calculateMotorPower(double forward, double strafe, double rotate) {

		// Mecanum calculations
		double leftFrontPower = forward + strafe - rotate;
		double rightFrontPower = forward - strafe + rotate;
		double leftRearPower = forward - strafe - rotate;
		double rightRearPower = forward + strafe + rotate;

		// Find magnitude
		double[] power = new double[]{rightFrontPower, leftFrontPower, rightRearPower, leftRearPower};
		double magnitude = Arrays.stream(power)
				.map(Math::abs).max()
				.orElse(1.0); // unreachable branch

		// Normalize if magnitude above 1.0
		if (magnitude > 1.0) {
			leftFrontPower /= magnitude;
			rightFrontPower /= magnitude;
			leftRearPower /= magnitude;
			rightRearPower /= magnitude;
		}

		return new DriveMotorConfig(rightFrontPower, leftFrontPower, rightRearPower, leftRearPower);

	}

}
