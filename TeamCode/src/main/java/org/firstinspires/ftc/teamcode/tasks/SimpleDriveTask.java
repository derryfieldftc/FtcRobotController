package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class SimpleDriveTask extends RobotTask {
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	DcMotor motorBL;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorFR;
	long millis;
	double y;
	double x;


	public SimpleDriveTask(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	public SimpleDriveTask time(long millis) {
		this.millis = millis;
		return this;
	}
	public SimpleDriveTask forward(double speed) {
		y = speed;
		return this;
	}
	public SimpleDriveTask strafe(double speed) {
		x = speed;
		return this;
	}

	public void init() {
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorBR = hardwareMap.dcMotor.get("motorBL");
		motorFR = hardwareMap.dcMotor.get("motorFR");

		motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
		motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

		motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	}

	@Override
	public void run() {
		double motorBLSpeed = y + x;
		double motorFLSpeed = y + x;
		double motorBRSpeed = y - x;
		double motorFRSpeed = y - x;

		motorBL.setPower(motorBLSpeed);
		motorFL.setPower(motorFLSpeed);
		motorBR.setPower(motorBRSpeed);
		motorFR.setPower(motorFRSpeed);

		opMode.sleep(millis);

		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
	}
}
