package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class CompSetup extends RobotTask {
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	Servo claw, wrist, elbow;

	public CompSetup(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
	}

	@Override
	public void init() {
		claw = hardwareMap.servo.get("claw");
		wrist = hardwareMap.servo.get("rotate");
		elbow = hardwareMap.servo.get("hinge");
		claw.setPosition(.85);
		wrist.setPosition(.95);
		elbow.setPosition(1);

	}
}
