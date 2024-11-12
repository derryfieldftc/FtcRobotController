package org.firstinspires.ftc.teamcode.tasks;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
	LinearOpMode opMode;

	double targetX;
	double targetY;
	double targetR;

	IMU imu;
	RevHubOrientationOnRobot orientationOnRobot;

	// for encoders
	double countsPerMotorRev = 8192;
	double wheelDiameterInches = 2.3;
	double countsPerInch = countsPerMotorRev / (wheelDiameterInches * Math.PI);


	public OdometryRunToPositionTask(LinearOpMode opMode, int xInches, int yInches) {
		this.opMode = opMode;
		this.targetX = xInches * countsPerInch;
		this.targetY = yInches * countsPerInch;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;

		imu = hardwareMap.get(IMU.class, "imu");
		RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
		RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
		RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
		imu.initialize(new IMU.Parameters(orientationOnRobot));

		this.targetR = Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
	}

	// Code is run ONCE when the driver hits INIT
	public void init() {
		this.strafeEncoder = hardwareMap.dcMotor.get("strafe");
		this.driveEncoder = hardwareMap.dcMotor.get("drive");
		this.imu = hardwareMap.get(IMU.class, "imu");
		imu.resetYaw();

		strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

	// Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
	public void run() {
		double errorDistance = 0;
		double acceptableRange = .3 * countsPerInch;

		do {
			double x = -strafeEncoder.getCurrentPosition();
			double y = -driveEncoder.getCurrentPosition();
			double r = Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

			double deltaX = targetX - x;
			double deltaY = targetY - y;
			double deltaR = targetR - r;

			double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
			deltaX = deltaX / magnitude;
			deltaY = deltaY / magnitude;

			double powerFL = deltaY + deltaX + deltaR;
			double powerBL = deltaY - deltaX + deltaR;
			double powerFR = deltaY - deltaX - deltaR;
			double powerBR = deltaY + deltaX - deltaR;

			double scale = errorDistance / acceptableRange;

			powerFL = clamp(1, -1, powerFL) * .3;
			powerBL = clamp(1, -1, powerBL) * .3;
			powerFR = clamp(1, -1, powerFR) * .3;
			powerBR = clamp(1, -1, powerBR) * .3;

			motorFL.setPower(powerFL);
			motorBL.setPower(powerBL);
			motorFR.setPower(powerFR);
			motorBR.setPower(powerBR);
			/*
			telemetry.addData("powerFR", powerFR);
			telemetry.addData("powerBR", powerBR);
			telemetry.addData("powerBL", powerBL);
			telemetry.addData("powerFL", powerFL);
			telemetry.update();
			 */

			updateTelemetry();
			telemetry.addData("error", errorDistance);

			errorDistance = magnitude;
		} while (errorDistance > acceptableRange && opMode.opModeIsActive());
	}

	public double calcError(double x, double y, double targetX, double targetY) {
		double deltaX = Math.abs(targetX - x);
		double deltaY = Math.abs(targetY - y);
		return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
	}

	public void updateTelemetry() {
		double x = strafeEncoder.getCurrentPosition();
		double y = driveEncoder.getCurrentPosition();
		telemetry.addData("xpos", x);
		telemetry.addData("xtar", targetX);
		telemetry.addData("ypos", y);
		telemetry.addData("ytar", targetY);
		telemetry.addLine("heading? " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		telemetry.update();
	}
}

