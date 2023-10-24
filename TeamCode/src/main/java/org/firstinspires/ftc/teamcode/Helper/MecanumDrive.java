package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {

	HardwareMap hardwareMap;

	DcMotor rightFront;
	DcMotor leftFront;
	DcMotor rightRear;
	DcMotor leftRear;

	// constructor that takes motor names as args.
	public MecanumDrive(
			HardwareMap origMap,
			String rightFrontName,
			String leftFrontName,
			String rightRearName,
			String leftRearName
	) {
		// get a reference to the hardware map for the robot.
		hardwareMap = origMap;

		// get references to the drive motors.
		rightFront = (DcMotor) hardwareMap.get(rightFrontName);
		leftFront = (DcMotor) hardwareMap.get(leftFrontName);
		rightRear = (DcMotor) hardwareMap.get(rightRearName);
		leftRear = (DcMotor) hardwareMap.get(leftRearName);

		// set motor directions.
		// for our mecanum drive let's reverse the left drive motors
		// so that a positive value will move the robot forwards.
		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		// set the behavior to brake mode for the drive motors.
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

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
	}

	public void drive(
			double forward,
			double strafe,
			double rotate,
			double scale
	) {
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

		this.leftFront.setPower(scale * leftFrontPower);
		this.rightFront.setPower(scale * rightFrontPower);
		this.leftRear.setPower(scale * leftRearPower);
		this.rightRear.setPower(scale * rightRearPower);

	}

	// override drive so scale is optional.
	public void drive(
			double forward,
			double strafe,
			double rotate
	) {
		this.drive(forward, strafe, rotate, 1.0);
	}
}
