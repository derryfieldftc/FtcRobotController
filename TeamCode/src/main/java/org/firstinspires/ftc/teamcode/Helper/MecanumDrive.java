package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.DriveMotorConfig;
import org.firstinspires.ftc.teamcode.Util.EncoderMotorConfig;

public class MecanumDrive {

    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;
	double encoderResolution;
	double wheelDiameter;
	LinearOpMode opMode;

	public MecanumDrive(
			DcMotor rightFront,
			DcMotor leftFront,
			DcMotor rightRear,
			DcMotor leftRear,
			double encoderResolution,
			double wheelDiameter,
			LinearOpMode opMode
	) {
		this.rightFront = rightFront;
		this.leftFront = leftFront;
		this.rightRear = rightRear;
		this.leftRear = leftRear;

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
			double encoderResolution,
			double wheelDiameter,
			LinearOpMode opMode
	) {
		rightFront = (DcMotor)hardwareMap.get(rightFrontMotorName);
		leftFront = (DcMotor)hardwareMap.get(leftFrontMotorName);
		rightRear = (DcMotor)hardwareMap.get(rightRearMotorName);
		leftRear = (DcMotor)hardwareMap.get(leftRearMotorName);

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
		opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
		opMode.telemetry.addData("Target Position", leftFront.getTargetPosition());
		opMode.telemetry.update();

		if (centimeters > 0) {
			while (opMode.opModeIsActive() && (leftFront.getCurrentPosition() <= leftFront.getTargetPosition())) {
				setMotorPowers(power);
				opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
				opMode.telemetry.addData("Target Position", leftFront.getTargetPosition());
				opMode.telemetry.update();
			}
			setMotorPowers(0);
		}
		else if (centimeters < 0) {
			while (opMode.opModeIsActive() && (leftFront.getCurrentPosition() >= leftFront.getTargetPosition())) {
				setMotorPowers(power);
				opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
				opMode.telemetry.addData("Target Position", leftFront.getTargetPosition());
				opMode.telemetry.update();
			}
			setMotorPowers(0);
		}
	}

	private void setMotorPowers(double power) {
		leftFront.setPower(power);
		rightFront.setPower(power);
		leftRear.setPower(power);
		rightRear.setPower(power);
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
		return new DriveMotorConfig(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
	}

	private static EncoderMotorConfig calculateTargetPosition(double resolution, double wheelDiameter, double centimeters) {
		double targetEncoderCounts = (centimeters * resolution) / (Math.PI * wheelDiameter);
		return new EncoderMotorConfig(targetEncoderCounts);
	}

}
