package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDClaw extends RobotPlugin {
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Telemetry telemetry;
	DcMotor pitch; //extended -6509 down 0
	DcMotor slide; //-26 start (hardwarestop) 121 pick
	Servo elbow; // clawP
	Servo rotate; //wrist????
	Servo claw; //claw
	GamepadManager gamepad;
	Servo bucket;
	int pitchTarget = 0;
	int slideTarget = 0;
	boolean clawOpen = false;

	/*
	 * claw OPEN = .83 CLOSED = .7
	 * wrist down 0 and up 1 dfk which is which yet
	 * elbow 1 closed / drop, nab .8
	 */

	public ITDClaw(LinearOpMode opMode) {
		this.opMode = opMode;
		this.telemetry = opMode.telemetry;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad = new GamepadManager(opMode.gamepad2);
	}

	public void init() {
		pitch = hardwareMap.dcMotor.get("pitch");
		slide = hardwareMap.dcMotor.get("slide");
		elbow = hardwareMap.servo.get("clawP");
		rotate = hardwareMap.servo.get("rotate");
		claw = hardwareMap.servo.get("claw");
		bucket = hardwareMap.servo.get("bucket"); //dumping = 0, grabbing = 1

		pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		pitch.setTargetPosition(pitchTarget);
		pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		pitch.setPower(1);

		slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slide.setTargetPosition(slideTarget);
		slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slide.setPower(1);

	}

	public void loop() {
		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) clawOpen = !clawOpen; //bad
		if (clawOpen) { claw.setPosition(.9); } else { claw.setPosition(.6); }

		telemetry.addData("claw open", clawOpen);

		if (gamepad.justPressed(GamepadManager.Button.A)) { //nabbing time
			pitchTarget = -26;
			pitch.setTargetPosition(pitchTarget); //unc onviable nbad
			elbow.setPosition(1);
			rotate.setPosition(1);
		}
		if (gamepad.justPressed(GamepadManager.Button.X)) { //placing
			pitchTarget = 121;
			pitch.setTargetPosition(pitchTarget);
			elbow.setPosition(.8);//wrong
			rotate.setPosition(1); //wrong
		}
		if (gamepad.pressed(GamepadManager.Button.DPAD_RIGHT)) pitchTarget++;
		if (gamepad.pressed(GamepadManager.Button.DPAD_LEFT)) pitchTarget--;
		telemetry.addData("pitch target", pitchTarget);

		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) {
			slide.setTargetPosition(-6509);
		}
		if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			slide.setTargetPosition(0);
		}
		if (gamepad.pressed(GamepadManager.Button.RIGHT_BUMPER)) { bucket.setPosition(0); } else { bucket.setPosition(1); }
		gamepad.poll();
	}
}
