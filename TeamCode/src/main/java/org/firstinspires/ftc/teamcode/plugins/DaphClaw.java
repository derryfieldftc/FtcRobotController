package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotPlugin;

public class DaphClaw extends RobotPlugin {

	enum Positions {
		VIPER,
		YOINK,
		IDLE,
	}
	/*
	 * State 1: Viper
	 * 	base = .45
	 * 	shoulder = 1
	 * 	elbow = 1
	 * 	bend = .1
	 * 	wrist = .55
	 * 	claw = 0
	 *
	 * State 2: yoink
	 * 	base = .45
	 *  shoulder = .35
	 * 	elbow = .55
	 * 	bend = .05
	 * 	wrist = .55
	 * 	claw = 0
	 *
	 */

	Servo base, shoulder, elbow, bend, wrist, claw;
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Gamepad gamepad;
	Positions mode = Positions.VIPER;

	public DaphClaw(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad = opMode.gamepad1;
	}

	public void init() {
		base = hardwareMap.servo.get("base");
		shoulder = hardwareMap.servo.get("shoulder");
		elbow = hardwareMap.servo.get("elbow");
		bend = hardwareMap.servo.get("bend");
		wrist = hardwareMap.servo.get("wrist");
		claw = hardwareMap.servo.get("claw");

	}

	public void loop() {
		if (gamepad.a) {
			mode = Positions.VIPER;
		} else if (gamepad.b) {
			mode = Positions.YOINK;
		} else if (gamepad.x) {
			mode = Positions.IDLE;
		}

		if (mode == Positions.VIPER) {
			base.setPosition(.45);
			shoulder.setPosition(1);
			elbow.setPosition(1);
			bend.setPosition(.1);
			wrist.setPosition(.55);
			claw.setPosition(0);

		}

		if (mode == Positions.YOINK) {
			base.setPosition(.45);
			shoulder.setPosition(.35);
			elbow.setPosition(.55);
			bend.setPosition(.05);
			wrist.setPosition(.55);
			claw.setPosition(0);

		}
	}
}

/* veymm`papa = harwadwareMap.get.DcSservo.get();%%pa"xbi"o
mp`meeeb

veymm`tp`meelv$y`t$a =.setPosition();bpomt`mjmm
 */