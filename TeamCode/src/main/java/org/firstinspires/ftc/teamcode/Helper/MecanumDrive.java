package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.robotcore.hardware.DcMotor;

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

		this.leftFront.setPower(  scale * leftFrontPower / magnitude );
		this.rightFront.setPower( scale * rightFrontPower / magnitude );
		this.leftRear.setPower(   scale * leftRearPower / magnitude );
		this.rightRear.setPower(  scale * rightRearPower / magnitude );

	}

}
