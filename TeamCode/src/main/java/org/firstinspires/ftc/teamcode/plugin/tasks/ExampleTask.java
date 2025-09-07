package org.firstinspires.ftc.teamcode.plugin.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.plugin.RobotTask;

public class ExampleTask extends RobotTask {
	Telemetry telemetry;
	OpMode opMode;
	HardwareMap hardwareMap;
	DcMotor motorFL;
	DcMotor motorFR;
	DcMotor motorBL;
	DcMotor motorBR;


	public ExampleTask(OpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
	}

	double speed = 0.0;
	public ExampleTask speed(double speed) {
		this.speed = speed;
		return this;
	}

	double seconds = 0.0;
	public ExampleTask seconds(double seconds) {
		this.seconds = seconds;
		return this;
	}

	public void init() {
		telemetry.addLine("This is an example autonomous task");
		telemetry.update();

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

	public void run() {
		double initalTime = opMode.getRuntime();
		double elapsedTime = 0;
		for (; elapsedTime < initalTime + seconds; elapsedTime = opMode.getRuntime()) {
			motorBL.setPower(speed);
			motorFL.setPower(speed);
			motorBR.setPower(speed);
			motorFR.setPower(speed);
		}
		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
	}
}
