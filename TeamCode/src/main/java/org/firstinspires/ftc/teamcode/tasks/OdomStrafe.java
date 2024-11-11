package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class OdomStrafe extends RobotTask {

	OpMode opMode;
	HardwareMap hardwareMap;
	DcMotor driveEncoder;
	Telemetry telemetry;

	public OdomStrafe(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		driveEncoder = hardwareMap.dcMotor.get("drive");
		driveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		driveEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void run() {
		while (true) {
			telemetry.addData("how faahr", driveEncoder.getCurrentPosition());
			telemetry.update();
		}
	}
}
