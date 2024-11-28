package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotPlugin;

/*
 * RETRACTED
 *   base =0.50
 *   shoulder = 1.00
 *   elbow = 1.00
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.50   // do not exceed 0.50.
 *
 * PREPICK
 *   base = 0.50
 *   shoulder = 0.21;
 *   elbow = 0.18
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.50
 *
 *   PREPICK2
 *   base = 0.50
 *   shoulder = 0.12
 *   elbow = 0.32
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.50
 *
 * PICK
 *   base = 0.50
 *   shoulder = 0.12
 *   elbow = 0.32
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.12
 *
 *
 * POSTPICK
 *   base = 0.50
 *   shoulder = 0.68
 *   elbow = 0.80
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.12
 *
 * DROP
 *   base = 0.50
 *   shoulder = 0.68
 *   elbow = 0.80
 *   bend = 0.50
 *   wrist = 0.00
 *   claw = 0.50
 *
 */
public class prevArmBot extends RobotPlugin {

	enum Positions {
		RETRACTED,
		PREPICK,
		PREPICK2,
		PICK,
		POSTPICK1,
		POSTPICK2,
		DROP
	}

	Servo base, shoulder, elbow, bend, wrist, claw;
	LinearOpMode opMode;
	HardwareMap hardwareMap;
	Gamepad gamepad;
	Positions mode = Positions.RETRACTED;

	boolean prevPressedRB = false;
	boolean clawOpen = false;
	boolean prevDpadUp = false;
	boolean prevDpadDown = false;

	public prevArmBot(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.gamepad = opMode.gamepad2;
	}

	public void init() {
		base = hardwareMap.servo.get("base");
		shoulder = hardwareMap.servo.get("shoulder");
		elbow = hardwareMap.servo.get("elbow");
		bend = hardwareMap.servo.get("bend");
		wrist = hardwareMap.servo.get("wrist");
		claw = hardwareMap.servo.get("claw");

	}

	public void nextMode() {
		if (mode == Positions.DROP) {
			mode = Positions.RETRACTED;
		} else if (mode == Positions.RETRACTED) {
			mode = Positions.PREPICK;
		} else if (mode == Positions.PREPICK) {
			mode = Positions.PREPICK2;
		}  else if (mode == Positions.PREPICK2) {
			mode = Positions.PICK;
		} else if (mode == Positions.PICK) {
			mode = Positions.POSTPICK1;
		} else if (mode == Positions.POSTPICK1) {
			mode = Positions.POSTPICK2;
		} else if (mode == Positions.POSTPICK2) {
			mode = Positions.DROP;
		}
	}

	public void prevMode() {
		if (mode == Positions.RETRACTED) {
			mode = Positions.DROP;
		} else if (mode == Positions.DROP) {
			mode = Positions.POSTPICK2;
		} else if (mode == Positions.POSTPICK2) {
			mode = Positions.POSTPICK1;
		} else if (mode == Positions.POSTPICK1) {
			mode = Positions.PICK;
		} else if (mode == Positions.PICK) {
			mode = Positions.PREPICK2;
		}  else if (mode == Positions.PREPICK2) {
			mode = Positions.PREPICK;
		}else if (mode == Positions.PREPICK) {
			mode = Positions.RETRACTED;
		}
	}

	public void loop() {
		if (gamepad.dpad_up && prevDpadUp == false) {
			prevDpadUp = true;
			prevMode();
		} else if (gamepad.dpad_up == false && prevDpadUp == true) {
			prevDpadUp = false;
		}

		if (gamepad.dpad_down && prevDpadDown == false) {
			prevDpadDown = true;
			nextMode();
		} else if (gamepad.dpad_down == false && prevDpadDown == true) {
			prevDpadDown = false;
		}

		if (mode == Positions.RETRACTED) {
			base.setPosition(.50);
			shoulder.setPosition(.3);
			elbow.setPosition(1);
			bend.setPosition(.5);
			wrist.setPosition(.00);
		} else if (mode == Positions.PREPICK) {
			base.setPosition(.50);
			shoulder.setPosition(.31);
			elbow.setPosition(.18);
			bend.setPosition(.5);
			wrist.setPosition(.2);
		} else if (mode == Positions.PICK) {
			base.setPosition(.50);
			shoulder.setPosition(.22);
			elbow.setPosition(.32);
			bend.setPosition(.5);
			wrist.setPosition(.2);
		} else if (mode == Positions.PREPICK2) {
			base.setPosition(.50);
			shoulder.setPosition(.22);
			elbow.setPosition(.32);
			bend.setPosition(.5);
			wrist.setPosition(.2);
		} else if (mode == Positions.POSTPICK1) {
			base.setPosition(.50);
			shoulder.setPosition(.45);
			elbow.setPosition(.32);
			bend.setPosition(.5);
			wrist.setPosition(.2);
		} else if (mode == Positions.POSTPICK2) {
			base.setPosition(.50);
			shoulder.setPosition(.75);
			elbow.setPosition(.75);
			bend.setPosition(.5);
			wrist.setPosition(.2);
		} else if (mode == Positions.DROP) {
			base.setPosition(.50);
			shoulder.setPosition(.75);
			elbow.setPosition(.80);
			bend.setPosition(.5);
			wrist.setPosition(.00);
		}

		//claw toggle
		if (gamepad.right_bumper && !prevPressedRB) {
			clawOpen = !clawOpen;
			prevPressedRB = true;
		} else if (!gamepad.right_bumper){
			prevPressedRB = false;
		}

		claw.setPosition((clawOpen) ? .5 : .12 );
	}
}
