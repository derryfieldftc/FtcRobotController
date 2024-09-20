package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

	public Odometry(OpMode opMode) {
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
	}

	// Code is run ONCE when the driver hits INIT
	public void init() {
		this.strafeEncoder = hardwareMap.dcMotor.get("strafeEncoder");
		this.driveEncoder = hardwareMap.dcMotor.get("driveEncoder");
	}

	// Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
	public void loop() {

	}

}
