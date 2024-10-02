package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class ExampleTask extends RobotTask {
	Telemetry telemetry;
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	DcMotor motorFL;
	DcMotor motorFR;
	DcMotor motorBL;
	DcMotor motorBR;


	public ExampleTask(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
	}

	double speed = 0.0;
	public ExampleTask speed(double speed) {
		this.speed = speed;
		return this;
	}

	long millis = 0;
	public ExampleTask seconds(long seconds) {
		this.millis = seconds;
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
		motorBL.setPower(speed);
		motorFL.setPower(speed);
		motorBR.setPower(speed);
		motorFR.setPower(speed);

		opMode.sleep(millis);

		motorBL.setPower(0);
		motorFL.setPower(0);
		motorBR.setPower(0);
		motorFR.setPower(0);
	}
}
