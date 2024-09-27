package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotTask;

/**
 * Odometry
 */


public class OdometryRunToPositionTask extends RobotTask {
	Telemetry telemetry;
	HardwareMap hardwareMap;
	DcMotor driveEncoder;
	DcMotor strafeEncoder;
	DcMotor motorFR;
	DcMotor motorBR;
	DcMotor motorFL;
	DcMotor motorBL;
	OpMode opMode;

	double targetX;
	double targetY;
	double r;


	IMU imu = null;
	RevHubOrientationOnRobot orientationOnRobot;

	//for encoders
	double countsPerMotorRev = 8192;
	double wheelDiameterInches = 2.3;
	double countsPerInch = countsPerMotorRev / (wheelDiameterInches * Math.PI);

	double desiredErrorInches = 1.5;

	public OdometryRunToPositionTask(OpMode opMode, int xInches, int yInches, int r) {
		this.opMode = opMode;
		this.targetX = xInches * countsPerInch;
		this.targetY = yInches * countsPerInch;
		this.r = r;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
		RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
		RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
	}

	// Code is run ONCE when the driver hits INIT
	public void init() {
		this.strafeEncoder = hardwareMap.dcMotor.get("strafe");
		this.driveEncoder = hardwareMap.dcMotor.get("drive");
		this.imu = hardwareMap.get(IMU.class, "imu");
		imu.resetYaw();
		driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

		updateTelemetry();
	}

	// Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
	public void run() {
		double errorDistance;
		do {
			double x = strafeEncoder.getCurrentPosition();
			double y = driveEncoder.getCurrentPosition();

			double dx = targetX - x;
			double dy = targetY - y;

			double magnitude = Math.sqrt(dx * dx + dy * dy);

			double unitX = 1 * dx / magnitude;
			double unitY = 1 * dy / magnitude;

			double powerFL = unitY + unitX;
			double powerBL = unitY - unitX;
			double powerFR = unitY - unitX;
			double powerBR = unitY + unitX;

			double maxPower = Math.max(Math.max(Math.max(powerFL, powerBL), powerBR), powerFR);
			double motorPower = 0.5 / maxPower;

			motorFL.setPower(powerFL * motorPower);
			motorBL.setPower(powerBL * motorPower);
			motorFR.setPower(powerFR * motorPower);
			motorBR.setPower(powerBR * motorPower);

			errorDistance = calcErrorSquared(x, y, targetX, targetY);

			telemetry.addData("powerBL", powerBL);
			telemetry.addData("powerBR", powerBR);
			telemetry.addData("powerFL", powerFL);
			telemetry.addData("powerFR", powerFR);
			updateTelemetry();

			if (!((LinearOpMode) opMode).opModeIsActive()) {break;}

		} while (errorDistance > Math.pow(countsPerInch * desiredErrorInches, 2));

	}

	public double calcErrorSquared(double x, double y, double targetX, double targetY) {
		double deltaX = Math.abs(targetX - x);
		double deltaY = Math.abs(targetY - y);
		return deltaX * deltaX + deltaY * deltaY;
	}

	public void updateTelemetry() {
		double x = strafeEncoder.getCurrentPosition();
		double y = driveEncoder.getCurrentPosition();
		telemetry.addData("xpos", x);
		telemetry.addData("xtar", targetX);
		telemetry.addData("ypos", y);
		telemetry.addData("ytar", targetY);
		// telemetry.addLine("heading? " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		telemetry.update();
	}

	private double clamp(double max, double min, double num) {
		if (num > max) {
			return max;
		} else if (num < min) {
			return min;
		} else {
			return num;
		}
	}
}

