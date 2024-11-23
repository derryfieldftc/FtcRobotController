package org.firstinspires.ftc.teamcode.plugins;

import android.media.audiofx.DynamicsProcessing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
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
	DigitalChannel slideSwitch;

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
		elbow = hardwareMap.servo.get("hinge");
		bucket = hardwareMap.servo.get("bucket");
		slideSwitch = hardwareMap.digitalChannel.get("slideLimit");

		slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slide.setTargetPosition(0);
		slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		slide.setPower(1);

		horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		horizontalSlide.setTargetPosition(0);
		horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		horizontalSlide.setPower(1);

	}


	public void loop() {
		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER)) toggleClaw();
		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER)) toggleWrist();

		bucket.setPosition(realGamepad.right_trigger);

		if (gamepad.justPressed(GamepadManager.Button.A)) grabbing();
		if (gamepad.justPressed(GamepadManager.Button.B)) outOfBucketWay();
		if (gamepad.justPressed(GamepadManager.Button.Y)) aboutToDrop();
		if (gamepad.justPressed(GamepadManager.Button.X)) grabbed();

//		horizontalSlide.setPower(realGamepad.left_stick_y / 10);
		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT)) horizontalSlideTargetPos = -3400;
		if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT)) horizontalSlideTargetPos = 3000;
		if (gamepad.justPressed(GamepadManager.Button.LEFT_STICK)) horizontalSlideTargetPos = 0; // like its weeiiirrrddd
		horizontalSlideTargetPos += (int) (-realGamepad.left_stick_y * 20);

		horizontalSlide.setTargetPosition(horizontalSlideTargetPos); // IT CANNOT JUST BE ME
																	 // actually i think past me was just going crazy, it looks normal now

		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) slideTargetPos = 5000;
		if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) slideTargetPos = 0;
		slideTargetPos += (int) (-realGamepad.right_stick_y * 10);

		elbow.setPosition(elbow.getPosition() + realGamepad.left_stick_x / 100);

		slide.setTargetPosition((slideSwitch.getState()) ? slide.getCurrentPosition() : slideTargetPos);
		if (slideSwitch.getState()) telemetry.addLine("Slide is down");

		telemetry.addData("slide pos", slide.getCurrentPosition());
		telemetry.addData("slide tar", slideTargetPos);
		telemetry.addData("horizontalSlide pos", horizontalSlide.getCurrentPosition());
		telemetry.addData("horizontalSlide tar", horizontalSlideTargetPos);
		gamepad.poll();
	}

	boolean clawOpen = false;
	public void toggleClaw() {
		telemetry.addLine("toggle claw");
		claw.setPosition((clawOpen) ? .85 : .57);
		clawOpen = !clawOpen;
	}

	boolean wristOut = false;
	public void toggleWrist() {
		telemetry.addLine("toggle wrist");
		wrist.setPosition((wristOut) ? .3 : .97);
		wristOut = !wristOut;
	}

	public void grabbing() {
		telemetry.addLine("grabbin");
		wrist.setPosition(.3);
		elbow.setPosition(.27);
	}
	public void grabbed() {
		telemetry.addLine("grabbed");
		wrist.setPosition(.9);
		elbow.setPosition(.35);
	}
	public void aboutToDrop() {
		telemetry.addLine("abt to drop");
		wrist.setPosition(.97);
		elbow.setPosition(1);
	}
	public void outOfBucketWay() {
		telemetry.addLine("out of bukt way");
		elbow.setPosition(.7);
	}

	/*
	1 grabbing
	claw = .9
	rotate = .9 // .3
	hinge = .9 // .4
	2 just grabbed
	hinge = .75
	3 abt to bucket
	rotate = .2 // .97
	hinge = .55 //.9
	4 buket
	hinge = .6
	rotate = .2
	5 clear
	hinge = .7 / maybe go back to 1
	& slide = 5k
	 */

	/*
	horizontal slide pos (dw i know where to start it`)
	out = 3000
	back -3400

	 */
}
