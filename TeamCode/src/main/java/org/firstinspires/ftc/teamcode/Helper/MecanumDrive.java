package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.DriveMotorConfig;
import org.firstinspires.ftc.teamcode.Util.EncoderMotorConfig;

import java.util.function.Predicate;

public class MecanumDrive {

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;
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

		while (targetNotReached.test(leftFront) && opMode.opModeIsActive()) {
			setMotorPowers(-actualPower, actualPower, actualPower, -actualPower);
			debugEncoderPositions(leftFront);
		}
		setMotorPowers(0);
	}

	public void turnUsingIMU(double degrees, double power) {
		imu.resetYaw();

		double actualPower = Math.abs(power) * Math.signum(degrees); // Negative if clockwise
		double errorAroundZero = 0.5;

		if (degrees > 0){
			double degrees1 = degrees < 180 ? degrees : 180;
			double degrees2 = degrees > 180 ? degrees - 360 : 0;

			while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= degrees1
					&& imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -errorAroundZero
					&& opMode.opModeIsActive()) {
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(degrees1, degrees2, imu);
			}
			while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= degrees2
					&& opMode.opModeIsActive()){
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(degrees1, degrees2, imu);
			}
		}
		if (degrees < 0){
			double degrees1 = degrees > -180 ? degrees : -180;
			double degrees2 = degrees < -180 ? degrees + 360 : 0;

			while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= degrees1
					&& imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < errorAroundZero
					&& opMode.opModeIsActive()) {
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(degrees1, degrees2, imu);
			}
			while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= degrees2
					&& opMode.opModeIsActive()){
				setMotorPowers(actualPower, -actualPower, actualPower, -actualPower);
				debugIMUYawDegrees(degrees1, degrees2, imu);
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
