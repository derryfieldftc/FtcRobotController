package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class SimpleDriveTask extends RobotTask {
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	IMU imu;
	DcMotor motorBL;
	DcMotor motorFL;
	DcMotor motorBR;
	DcMotor motorFR;
	long millis;
	double y = 0;
	double x = 0;
	double r = 0;


	public SimpleDriveTask(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		this.imu = hardwareMap.get(IMU.class, "imu");
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
	public SimpleDriveTask turn(double degrees) {

		return this;
	}

	public void init() {
		motorBL = hardwareMap.dcMotor.get("motorBL");
		motorFL = hardwareMap.dcMotor.get("motorFL");
		motorBR = hardwareMap.dcMotor.get("motorBR");
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
		double motorBLSpeed = y - x + r;
		double motorFLSpeed = y + x + r;
		double motorBRSpeed = y + x - r;
		double motorFRSpeed = y - x - r;

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
