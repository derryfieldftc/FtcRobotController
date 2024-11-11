package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.DebuggableReentrantLock;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;

/**
 * Odometry
 */


public class Odometry extends RobotPlugin {
	Telemetry telemetry;
	HardwareMap hardwareMap;
	DcMotor driveEncoder;
	DcMotor strafeEncoder;
	IMU imu = null;
	RevHubOrientationOnRobot orientationOnRobot;

	static final double     COUNTS_PER_MOTOR_REV    = 0; //TODO!
	static final double     DRIVE_GEAR_REDUCTION    = 1;
	static final double     WHEEL_DIAMETER_INCHES   = 2;
	static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
			(WHEEL_DIAMETER_INCHES * 3.1415);
	static final double     DRIVE_SPEED             = .6;
	static final double     TURN_SPEED              = .5;

	public Odometry(OpMode opMode) {
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
	}

	// Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
	public void loop() {
		telemetry.addLine("strafe: " + strafeEncoder.getCurrentPosition());
		telemetry.addLine("drive: " + driveEncoder.getCurrentPosition());
		telemetry.addLine("heading? " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		telemetry.update();
	}

}
