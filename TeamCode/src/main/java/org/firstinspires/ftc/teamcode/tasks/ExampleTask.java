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

	public ExampleTask(OpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
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
	}

	public RobotTask parameters(double speed, double seconds) {
		this.speed = speed;
		this.seconds = seconds;
		return this;
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
