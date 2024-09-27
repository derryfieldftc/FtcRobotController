package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

import java.time.Duration;

public class ExampleTask extends RobotTask {
	Telemetry telemetry;
	OpMode opMode;
	HardwareMap hardwareMap;
	DcMotor motorFL;
	DcMotor motorFR;
	DcMotor motorBL;
	DcMotor motorBR;
	double speed;
	double seconds;

	public ExampleTask(OpMode opMode, double speed, double seconds) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
		this.speed = speed;
		this.seconds = seconds;
	}

	public void init() {
		telemetry.addLine("This is an example autonomous task");
		telemetry.update();

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
	}

	public void run() {
		double initalTime = opMode.getRuntime();
		double elapsedTime = 0;
		while (elapsedTime < initalTime + seconds) {
			motorBL.setPower(speed);
			motorFL.setPower(speed);
			motorBR.setPower(speed);
			motorFR.setPower(speed);
			elapsedTime = opMode.getRuntime();
		}
		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
	}
}
