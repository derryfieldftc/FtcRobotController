package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDClaw extends RobotPlugin {
	public HardwareMap hardwareMap;
	public OpMode opMode;
	public GamepadManager gamepad;
	public Gamepad realGamepad;
	public Telemetry telemetry;

	private DcMotor shoulder;
	private DcMotor slide;
	private Servo claw;
	private Servo bucket;
	private Servo wrist;
	private Servo tilt;
	private Servo elbow;

	private int shouldPos;
	private int slidePos;

	public void init() {
		shoulder = hardwareMap.dcMotor.get("shoulder");
		slide = hardwareMap.dcMotor.get("slide");
		claw = hardwareMap.servo.get("claw");
		bucket = hardwareMap.servo.get("bucket");
		wrist = hardwareMap.servo.get("wrist");
		tilt = hardwareMap.servo.get("tilt");
		elbow = hardwareMap.servo.get("elbow");

		shoulder.setTargetPosition(shoulder.getCurrentPosition());
		shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		shoulder.setPower(0);

		slide.setTargetPosition(slide.getCurrentPosition());
		slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slide.setPower(0);

		shouldPos = shoulder.getCurrentPosition();
		slidePos = slide.getCurrentPosition();


//		tuck();
		bucketUp();
		halfFold();

	}

	public void loop() {
		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) {
			slidePos = -6509;
		}
		if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			slidePos = 0;
		}



		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER))
			toggleWrist();
		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER))
			toggleClaw();

		shouldPos += -realGamepad.right_stick_y * 5;
		slidePos += -realGamepad.left_stick_y * 5;
		telemetry.addData("shoulderPos", shouldPos);

		bucket.setPosition(realGamepad.right_trigger + .55);

		if (gamepad.justPressed(GamepadManager.Button.A))
			fullFold();
		if (gamepad.justPressed(GamepadManager.Button.B))
			transfer();
		gamepad.poll();

		if (gamepad.justPressed(GamepadManager.Button.DPAD_LEFT))
			tilt.setPosition(.8);
		if (gamepad.justPressed(GamepadManager.Button.DPAD_RIGHT))
			tilt.setPosition(1);

		if (gamepad.justPressed(GamepadManager.Button.LEFT_STICK)) {
			shoulder.setPower(.75);
			slide.setPower(1);
		}


		shoulder.setTargetPosition(shouldPos);
		slide.setTargetPosition(slidePos);

	}

	public ITDClaw(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		realGamepad = opMode.gamepad2;
		this.gamepad = new GamepadManager(realGamepad);
		this.telemetry = opMode.telemetry;

	}

	public void tuck() {
		bucket.setPosition(.55);
		shouldPos = 0;
		tilt.setPosition(.16);
		wrist.setPosition(0);
		elbow.setPosition(0);
		claw.setPosition(.77);
	}

	public void bucketUp() {
		bucket.setPosition(1);
	}

	public void halfFold() {
		shouldPos = -2900;
	}

	public void fullFold() {
		shouldPos = -5800;
		elbow.setPosition(1);
		tilt.setPosition(.75);
		wrist.setPosition(0);
	}

	public void transfer() {
		bucket.setPosition(.55);
		tilt.setPosition(0);
		wrist.setPosition(.25);
		shouldPos = -2325;
	}

	boolean clawOpen = false;
	public void toggleClaw() {
		claw.setPosition((clawOpen) ? .77 : 1);
		clawOpen = !clawOpen;
	}

	boolean wristRotated = false;
	public void toggleWrist() {
		wrist.setPosition((wristRotated) ? 0 : .75);
		wristRotated = !wristRotated;
	}

}

/*		motorStateMachine = new MotorStateMachine.Builder()
				.addMotor(shoulder)
				.addMotor(slide)
				.addServo(claw)
				.addServo(bucket)
				.addServo(wrist)
				.addServo(tilt)
				.addServo(elbow)
				.addState("tuck",
						StateActionPrototype.servoTarget(bucket, .55),
						StateActionPrototype.servoTarget(wrist, .16),
						StateActionPrototype.servoTarget(tilt, .16),
						StateActionPrototype.servoTarget(elbow, 0))
				.addState("fold_1",
						StateActionPrototype.motorTarget(shoulder, 1, -2900))
				.addState("fold_2",
						StateActionPrototype.servoTarget(elbow, 1),
						StateActionPrototype.servoTarget(tilt, .65),
						StateActionPrototype.servoTarget(wrist, 0))
				.build(hardwareMap);


 */
