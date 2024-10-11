package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotTask;

public class OdomStrafe extends RobotTask {

	double countsPerMotorRev = 8192;
	double wheelDiameterInches = 2.3;
	double countsPerInch = countsPerMotorRev / (wheelDiameterInches * Math.PI);
	double errorInches = 1;
	double speed = .5;
	double target;
	OpMode opMode;
	HardwareMap hardwareMap;
	DcMotor strafeEncoder;
	DcMotor motorFR;
	DcMotor motorBR;
	DcMotor motorFL;
	DcMotor motorBL;

	public OdomStrafe(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
	}

	public void init() {
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorFR = hardwareMap.dcMotor.get("motorFR");
		motorBR = hardwareMap.dcMotor.get("motorBR");

		motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
		motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

		strafeEncoder = hardwareMap.dcMotor.get("strafe");
		strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public OdomStrafe speed(double speed) {
		if (speed > 0 && speed <= 1)
			this.speed = speed;
		return this;
	}

	public OdomStrafe error(double errorInches) {
		this.errorInches = errorInches * countsPerInch;
		return this;
	}

	public OdomStrafe target(double targetInches) {
		this.target = -targetInches * countsPerInch;
		return this;
	}

	public void run() {
		double powerFL;
		double powerBL;
		double powerFR;
		double powerBR;
		double error;
		do {
			double currentPos = strafeEncoder.getCurrentPosition();
			if (target < 0) {
				powerFL = 1;
				powerBL = -1;
				powerFR = -1;
				powerBR = 1;
			} else {
				powerFL = -1;
				powerBL = 1;
				powerFR = 1;
				powerBR = -1;
			}

			motorFL.setPower(powerFL * speed);
			motorBL.setPower(powerBL * speed);
			motorFR.setPower(powerFR * speed);
			motorBR.setPower(powerBR * speed);

			error = target - currentPos;
		} while (error > errorInches);
	}
}
