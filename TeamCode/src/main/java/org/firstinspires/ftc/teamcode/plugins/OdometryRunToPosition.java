package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotPlugin;

/**
 * Odometry
 */


public class OdometryRunToPosition extends RobotPlugin {
	Telemetry telemetry;
	HardwareMap hardwareMap;
	DcMotor driveEncoder;
	DcMotor strafeEncoder;
	DcMotor motorFR;
	DcMotor motorBR;
	DcMotor motorFL;
	DcMotor motorBL;

	IMU imu = null;
	RevHubOrientationOnRobot orientationOnRobot;

	//for encoders
	double countsPerMotorRev = 0; //TODO!
	double wheelDiameterInches = 2;
	double countsPerInch = countsPerMotorRev / (wheelDiameterInches * 3.1415);

	// for actual spinny motors
	double wheelSize;
	double rpm;
	double driveSpeed = .6;
	double turnSpeed = .5;



	public OdometryRunToPosition(OpMode opMode) {
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
	public void loop() {
		telemetry.addLine("strafe: " + strafeEncoder.getCurrentPosition());
		telemetry.addLine("drive: " + driveEncoder.getCurrentPosition());
		telemetry.addLine("heading? " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		telemetry.update();
	}

	public void runToPosition(double x, double y, double speed) {


	}

	class Position {
		public int y;
		public int r;
		public int x;
		public Position(int x, int y, int r) {
			this.y = y;
			this.x = x;
			this.r = r;
		}
	}
}

