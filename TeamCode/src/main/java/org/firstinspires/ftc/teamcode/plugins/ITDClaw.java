package org.firstinspires.ftc.teamcode.plugins;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.RobotPlugin;

public class ITDClaw extends RobotPlugin {
	public HardwareMap hardwareMap;
	public OpMode opMode;
	public GamepadManager gamepad;
	public Gamepad realGamepad;

	private DcMotor shoulder;
	private DcMotor slide;
	private Servo claw;
	private Servo bucket;
	private Servo wrist;
	private Servo tilt;
	private Servo elbow;

	public void init() {
		shoulder = hardwareMap.dcMotor.get("shoulder");
		slide = hardwareMap.dcMotor.get("slide");
		claw = hardwareMap.servo.get("claw");
		bucket = hardwareMap.servo.get("bucket");
		wrist = hardwareMap.servo.get("wrist");
		tilt = hardwareMap.servo.get("tilt");
		elbow = hardwareMap.servo.get("elbow");

		shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		shoulder.setPower(.5);

		slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		slide.setPower(.1);

		tuck();
		bucketUp();
		halfFold();

	}

	public void loop() {
		if (gamepad.justPressed(GamepadManager.Button.DPAD_UP)) {
			slide.setTargetPosition(-6509);
		}
		if (gamepad.justPressed(GamepadManager.Button.DPAD_DOWN)) {
			slide.setTargetPosition(0);
		}

		if (gamepad.justPressed(GamepadManager.Button.RIGHT_BUMPER))
			toggleWrist();
		if (gamepad.justPressed(GamepadManager.Button.LEFT_BUMPER))
			toggleClaw();

		bucket.setPosition(realGamepad.right_trigger);

		if (gamepad.justPressed(GamepadManager.Button.A))
			fullFold();
		if (gamepad.justPressed(GamepadManager.Button.B))
			transfer();

	}

	public ITDClaw(OpMode opMode) {
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		realGamepad = opMode.gamepad2;
		this.gamepad = new GamepadManager(realGamepad);

	}

	public void tuck() {
		bucket.setPosition(.55);
		shoulder.setTargetPosition(0);
		tilt.setPosition(.16);
		wrist.setPosition(0);
		elbow.setPosition(0);
		claw.setPosition(.77);
	}

	public void bucketUp() {
		bucket.setPosition(1);
	}

	public void halfFold() {
		shoulder.setTargetPosition(-2900);
	}

	public void fullFold() {
		shoulder.setTargetPosition(-5800);
		elbow.setPosition(1);
		tilt.setPosition(.65);
		wrist.setPosition(0);
	}

	public void transfer() {
		bucket.setPosition(.55);
		tilt.setPosition(0);
		wrist.setPosition(.25);
		shoulder.setPower(-2325);
	}

	boolean clawOpen = false;
	public void toggleClaw() {
		claw.setPosition((clawOpen) ? .77 : 0);
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
