package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.DriveMotorConfig;

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

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightFront.setTargetPosition(0);
		leftFront.setTargetPosition(0);
		rightRear.setTargetPosition(0);
		leftRear.setTargetPosition(0);

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
		double targetEncoderTicks = (centimeters * encoderResolution)/(Math.PI * wheelDiameter);
		leftFront.setTargetPosition((int)(leftFront.getCurrentPosition() + targetEncoderTicks));
		rightFront.setTargetPosition((int)(rightFront.getCurrentPosition() + targetEncoderTicks));
		leftRear.setTargetPosition((int)(leftRear.getCurrentPosition() + targetEncoderTicks));
		rightRear.setTargetPosition((int)(rightRear.getCurrentPosition() + targetEncoderTicks));
		if(centimeters > 0){
			while(opMode.opModeIsActive() && (leftFront.getCurrentPosition() <= leftFront.getTargetPosition())) {
				drive(targetEncoderTicks, 0, power);

				opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
				opMode.telemetry.addData("Target Position", leftFront.getTargetPosition());
				opMode.telemetry.update();
			}
			drive(0, 0, 0);
		}
		if(centimeters < 0){
			while(opMode.opModeIsActive() && (leftFront.getCurrentPosition() >= leftFront.getTargetPosition())) {
				drive(targetEncoderTicks, 0, power);

				opMode.telemetry.addData("Current Position", leftFront.getCurrentPosition());
				opMode.telemetry.addData("Target Position", leftFront.getTargetPosition());
				opMode.telemetry.update();
			}
			drive(0, 0, 0);
		}

	}

	/**
	 * A blocking function that strafes a specified number of centimeters.
	 */
	public void driveCentimetersStrafe(double centimeters, double power) {
		double targetEncoderTicks = (centimeters * encoderResolution)/(Math.PI * wheelDiameter);
		leftFront.setTargetPosition((int)(leftFront.getCurrentPosition() - targetEncoderTicks));
		rightFront.setTargetPosition((int)(rightFront.getCurrentPosition() + targetEncoderTicks));
		leftRear.setTargetPosition((int)(leftRear.getCurrentPosition() - targetEncoderTicks));
		rightRear.setTargetPosition((int)(rightRear.getCurrentPosition() + targetEncoderTicks));

		if(centimeters > 0){
			while(opMode.opModeIsActive() && (leftFront.getCurrentPosition() <= leftFront.getTargetPosition())) {
				drive(targetEncoderTicks, 0, power);
			}
			drive(0, 0, 0);
		}
		if(centimeters < 0){
			while(opMode.opModeIsActive() && (leftFront.getCurrentPosition() >= leftFront.getTargetPosition())) {
				drive(targetEncoderTicks, 0, power);
			}
			drive(0, 0, 0);
		}
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

}
