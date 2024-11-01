package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDclaw extends RobotPlugin {
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	DcMotor pitch;
	Servo elbow; // clawP
	Servo rotate; //wrist????
	Servo claw; //claw
	GamepadManager gamepad;
	Servo bucket;
	int pitchTarget = 0;

	/*
	 * claw OPEN = .83 CLOSED = .7
	 * wrist 0 and 1 dfk which is which yet
	 * elbow 1 closed / drop, nab .8
	 */

	public ITDclaw(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad = new GamepadManager(opMode.gamepad2);
	}

	public void init() {
		pitch = hardwareMap.dcMotor.get("pitch");
		elbow = hardwareMap.servo.get("clawP");
		rotate = hardwareMap.servo.get("rotate");
		claw = hardwareMap.servo.get("claw");
		bucket = hardwareMap.servo.get("bucket"); //dumping = 0, grabbing = 1

		pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		pitch.setTargetPosition(pitchTarget);
		pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	}

	public void loop() {

		if (gamepad.pressed(GamepadManager.Button.A)) pitchTarget++;
		if (gamepad.pressed(GamepadManager.Button.B)) pitchTarget--;

		pitch.setTargetPosition(pitchTarget);
		pitch.setPower(1);

		telemetry.addData("pitch target", pitchTarget);
		telemetry.addData("pitch", pitch.getCurrentPosition());
		gamepad.poll();
	}
}
