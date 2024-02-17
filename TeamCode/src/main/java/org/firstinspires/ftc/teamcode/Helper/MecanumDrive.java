package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.DriveMotorConfig;
import org.firstinspires.ftc.teamcode.Util.EncoderMotorConfig;

import java.util.function.Predicate;

public class MecanumDrive {

    public DcMotor rightFront;
    public DcMotor leftFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
	IMU imu;
	double encoderResolution;
	double wheelDiameter;
	LinearOpMode opMode;

	public MecanumDrive(
			DcMotor rightFront,
			DcMotor leftFront,
			DcMotor rightRear,
			DcMotor leftRear,
			IMU imu,
			double encoderResolution,
			double wheelDiameter,
			LinearOpMode opMode
	) {
		this.rightFront = rightFront;
		this.leftFront = leftFront;
		this.rightRear = rightRear;
		this.leftRear = leftRear;
		this.imu = imu;
		this.encoderResolution = encoderResolution;
		this.wheelDiameter = wheelDiameter;
		this.opMode = opMode;
	}

	public MecanumDrive(
			HardwareMap hardwareMap,
			String rightFrontMotorName,
			String leftFrontMotorName,
			String rightRearMotorName,
			String leftRearMotorName,
			String imuName,
			double encoderResolution,
			double wheelDiameter,
			LinearOpMode opMode
	) {
		rightFront = (DcMotor)hardwareMap.get(rightFrontMotorName);
		leftFront = (DcMotor)hardwareMap.get(leftFrontMotorName);
		rightRear = (DcMotor)hardwareMap.get(rightRearMotorName);
		leftRear = (DcMotor)hardwareMap.get(leftRearMotorName);
		imu = (IMU)hardwareMap.get(imuName);

		rightFront.setDirection(DcMotor.Direction.FORWARD);
		leftFront.setDirection(DcMotor.Direction.REVERSE);
		rightRear.setDirection(DcMotor.Direction.FORWARD);
		leftRear.setDirection(DcMotor.Direction.REVERSE);

		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
		RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

		RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

		imu.initialize( new IMU.Parameters(orientationOnRobot));
		this.encoderResolution = encoderResolution;
		this.wheelDiameter = wheelDiameter;
		this.opMode = opMode;
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
	 * A blocking function that moves forward a specified number of centimeters.
	 */
	public void driveCentimetersForward(double centimeters, double power) {
		calculateTargetPosition(encoderResolution, wheelDiameter, centimeters)
				.addForwardTargetTo(rightFront, leftFront, rightRear, leftRear);

		double actualPower = Math.abs(power) * Math.signum(centimeters); // Negative if backwards

		Predicate<DcMotor> targetNotReached = (centimeters > 0) ?
				(motor -> motor.getCurrentPosition() <= motor.getTargetPosition()) :
				(motor -> motor.getCurrentPosition() >= motor.getTargetPosition());

		// Check if one motor (leftFront in this case) has reached its target
		while (targetNotReached.test(leftFront) && opMode.opModeIsActive()) {
			setMotorPowers(actualPower);
			debugEncoderPositions(leftFront);
		}
		setMotorPowers(0);
	}

	public void driveCentimetersStrafe(double centimeters, double power) {
		calculateTargetPosition(encoderResolution, wheelDiameter, centimeters)
				.addStrafeTargetTo(rightFront, leftFront, rightRear, leftRear);

		double actualPower = Math.abs(power) * Math.signum(centimeters); // Negative if backwards

		Predicate<DcMotor> targetNotReached = (centimeters > 0) ?
				(motor -> motor.getCurrentPosition() <= motor.getTargetPosition()) :
				(motor -> motor.getCurrentPosition() >= motor.getTargetPosition());

		// Check if one motor (leftFront in this case) has reached its target
		while (targetNotReached.test(leftFront) && opMode.opModeIsActive()) {
			setMotorPowers(-actualPower, actualPower, actualPower, -actualPower);
			debugEncoderPositions(leftFront);
		}
		setMotorPowers(0);
	}

	/**
	 * Blocking function that rotates the robot by a specified number of degrees.
	 */
	public void turnUsingIMU(double degrees, double power) {
		imu.resetYaw();

		double direction = Math.signum(degrees);
		double actualPower = Math.abs(power) * direction; // Negative if clockwise
		double errorAroundZero = 0.5;

		// How the IMU Handles Rotations
		//               0
		//             + | -
		//              180

		// Handle degrees <= 180
		if (Math.abs(degrees) <= 180) {
			while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) *
					direction <= degrees * direction && opMode.opModeIsActive()) {
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(direction, degrees, imu);
			}
		}
		// Handle degrees > 180
		else {
			// Turn 180 degrees as many times as we need
			int numberOf180DegreeTurns = (int)Math.floor(Math.abs(degrees) / 180);
			for (int i = 0; i < numberOf180DegreeTurns; i ++) {
				// Rotate 180 degrees; errorAroundZero is used because imu may be imprecise near 0
				if (i % 2 != 0) {
					while (Math.signum(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
							+ (errorAroundZero * direction)) == direction && opMode.opModeIsActive()) {
						setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
						debugIMUYawDegrees(direction, 180 * direction, imu);
					}
				}
				else {
					while (Math.signum(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
							+ (errorAroundZero * -direction)) == -direction && opMode.opModeIsActive()) {
						setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
						debugIMUYawDegrees(direction, 180 * -direction, imu);
					}
				}
			}
			double excessDegrees = (degrees % 180);
			// Rotate the excess degrees
			while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) * direction
					< excessDegrees * direction && opMode.opModeIsActive()) {
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(direction, (degrees - (360 * direction)), imu);
			}
		}
		setMotorPowers(0);
	}


	private void setMotorPowers(double power) {
		rightFront.setPower(power);
		leftFront.setPower(power);
		rightRear.setPower(power);
		leftRear.setPower(power);
	}
	private void setMotorPowers(double frontRight, double frontLeft, double backRight, double backLeft) {
		rightFront.setPower(frontRight);
		leftFront.setPower(frontLeft);
		rightRear.setPower(backRight);
		leftRear.setPower(backLeft);
	}


	private static DriveMotorConfig calculateMotorPower(double forward, double strafe, double rotate) {

		// Mecanum calculations
		double leftFrontPower = forward + strafe - rotate;
		double rightFrontPower = forward - strafe + rotate;
		double leftRearPower = forward - strafe - rotate;
		double rightRearPower = forward + strafe + rotate;

		double magnitude = Math.max(Math.max(Math.max(
			Math.abs(leftFrontPower),
			Math.abs(rightFrontPower)),
			Math.abs(leftRearPower)),
			Math.abs(rightRearPower)
		);

		if (magnitude > 1.0) {
			leftFrontPower /= magnitude;
			rightFrontPower /= magnitude;
			leftRearPower /= magnitude;
			rightRearPower /= magnitude;
		}
		return new DriveMotorConfig(rightFrontPower, leftFrontPower, rightRearPower, leftRearPower);
	}

	private static EncoderMotorConfig calculateTargetPosition(double resolution, double wheelDiameter, double centimeters) {
		double targetEncoderCounts = (centimeters * resolution) / (Math.PI * wheelDiameter);
		return new EncoderMotorConfig(targetEncoderCounts);
	}

	private void debugEncoderPositions(DcMotor motor) {
		opMode.telemetry.addData(motor.getDeviceName() + " Current Position", motor.getCurrentPosition());
		opMode.telemetry.addData(motor.getDeviceName() + " Target Position", motor.getTargetPosition());
		opMode.telemetry.update();
	}

	private void debugIMUYawDegrees(double degrees1, double degrees2, IMU imu) {
		opMode.telemetry.addData("Degree1", degrees1);
		opMode.telemetry.addData("Degree2", degrees2);
		opMode.telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		opMode.telemetry.update();
	}

}
