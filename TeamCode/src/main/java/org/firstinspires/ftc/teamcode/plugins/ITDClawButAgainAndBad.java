package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDClawButAgainAndBad extends RobotPlugin {

	LinearOpMode opMode;
	Telemetry telemetry;
	HardwareMap hardwareMap;

	DcMotor slide;
	DcMotor horizontalSlide; // is it just me or does that first lowercase h look wrong
	Servo claw;
	Servo wrist;
	Servo elbow;
	Servo bucket;
	Gamepad realGamepad;
	GamepadManager gamepad;

	int horizontalSlideTargetPos = 0; // this looks weird for the same reasons
	int slideTargetPos = 0;

	public ITDClawButAgainAndBad(LinearOpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		realGamepad = opMode.gamepad2;
		gamepad = new GamepadManager(opMode.gamepad2);

		telemetry.addData("loaded", this.getClass().toString());
	}

	public void init() {
		slide = hardwareMap.dcMotor.get("slide");
		horizontalSlide = hardwareMap.dcMotor.get("slideH"); // like i know it follows the style but like
		claw = hardwareMap.servo.get("claw");
		wrist = hardwareMap.servo.get("rotate");
		elbow = hardwareMap.servo.get("elbow");
		bucket = hardwareMap.servo.get("bucket");

		slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

	}


	public void loop() {
		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) toggleClaw();
		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) toggleWrist();

		bucket.setPosition((realGamepad.right_trigger > .5) ? 1 : 0);

		if (gamepad.justPressed(GamepadManager.Button.A)) grabbing();
		if (gamepad.justPressed(GamepadManager.Button.B)) outOfBucketWay();
		if (gamepad.justPressed(GamepadManager.Button.Y)) aboutToDrop();
		if (gamepad.justPressed(GamepadManager.Button.X)) grabbed();

		if (gamepad.justPressed(GamepadManager.Button.LEFT_STICK)) horizontalSlideTargetPos = 0; // like its weeiiirrrddd
		horizontalSlideTargetPos += (int) (realGamepad.left_stick_y * 50);

		horizontalSlide.setTargetPosition(horizontalSlideTargetPos); // IT CANNOT JUST BE ME

		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) slideTargetPos = 5000;
		if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) slideTargetPos = 0;
		slideTargetPos += (int) (realGamepad.right_stick_y * 10);

		slide.setTargetPosition(slideTargetPos);
	}

	boolean clawOpen = false;
	public void toggleClaw() {
		claw.setPosition((clawOpen) ? .9 : .35);
		clawOpen = !clawOpen;
	}

	boolean wristOut = false;
	public void toggleWrist() {
		wrist.setPosition((wristOut) ? .9 : .2);
		wristOut = !wristOut;
	}

	public void grabbing() {
		wrist.setPosition(.9);
		elbow.setPosition(.9);
	}
	public void grabbed() {
		wrist.setPosition(.9);
		elbow.setPosition(.75);
	}
	public void aboutToDrop() {
		wrist.setPosition(.2);
		elbow.setPosition(.55);
	}
	public void outOfBucketWay() {
		elbow.setPosition(.7);
	}

	/*
	1 grabbing
	claw = .9
	rotate = .9
	hinge = .9
	2 just grabbed
	hinge = .75
	3 abt to bucket
	rotate = .2
	hinge = .55
	4 buket
	hinge = .6
	rotate = .2
	5 clear
	hinge = .7 / maybe go back to 1
	& slide = 5k
	 */
}
