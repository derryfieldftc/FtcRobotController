package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;
import static org.firstinspires.ftc.teamcode.robot.Field.motif;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 * If you run into any null pointers check your enabled parts of the robot
 */
public class Robot extends RobotPart {
	public Drivetrain drivetrain;
	public boolean drivetrainEnabled;
	public Intake intake;
	public boolean intakeEnabled;
//	public Camera camera;
//	public boolean cameraEnabled;
	public Turret turret;
	public boolean turretEnabled;
	public HandsOfGod handsOfGod;
	public boolean handsOfGodEnabled;
	public PalmsOfGod palmsOfGod;
	public boolean palmsOfGodEnabled;
	public TurretPose2d finalPose;


	/**
	 * 1
	 * 3   2
	 */
	// lol final = immutable yeah sure man
	public static final Ball[] balls = {None, None, None};
	public static Ball handBall = balls[0];
	public static Ball rightBall = balls[1];
	public static Ball leftBall = balls[2];

	public enum BallPosition {
		Hands(handBall),
		Right(rightBall),
		Left(leftBall);

		public final Ball ball = None;

		BallPosition(Ball ball) {
		}

		public Ball getBall() {
			return this.ball;
		}
	}

	/**
	 * Do not forget to chain this with all of the enable methods
	 *
	 * @param opMode
	 */
	public Robot(OpMode opMode) {
		super(opMode);
		drivetrain = new Drivetrain(hardwareMap, this.opMode);
		intake = new Intake(this.opMode);
//		camera = new Camera(this.opMode);
		turret = new Turret(this.opMode, new TurretPose2d(new Pose(0, 0, 0), 0));
		handsOfGod = new HandsOfGod(this.opMode);
		palmsOfGod = new PalmsOfGod(this.opMode);
		voltageSensor = hardwareMap.voltageSensor.iterator()
				.next(); // funky but also how RR gets voltage sensor
	}

	public Robot enablePalmsOfGod() {
		palmsOfGodEnabled = true;
		return this;
	}

	public Robot enableHandsOfGod() {
		handsOfGodEnabled = true;
		return this;
	}

	public Robot enableDriveTrain() {
		drivetrainEnabled = true;
		return this;
	}

	public Robot enableTurret() {
		turretEnabled = true;
		return this;
	}

	public Robot enableIntake() {
		intakeEnabled = true;
		return this;
	}

//	public Robot enableIntakeSpinner() {
//		intakeSpinnerEnabled = true;
//		return this;
//	}

	public double getVoltage() {
		return voltageSensor.getVoltage();
	}

//	public Robot enableCamera() {
//		cameraEnabled = true;
//		return this;
//	}

	public void init() {
		if (intakeEnabled)
			intake.init();
//		if (intakeSpinnerEnabled)
//			intakeSpinner.init();
//		if (cameraEnabled)
//			camera.init();
		if (turretEnabled)
			turret.init();
		if (handsOfGodEnabled)
			handsOfGod.init();
		if (palmsOfGodEnabled)
			palmsOfGod.init();
	}

	double currentTime = 0;

	public void loop() {
		currentTime = opMode.getRuntime();
		if (intakeEnabled)
			intake.loop();
//		if (cameraEnabled)
//			camera.loop();
		if (turretEnabled)
			turret.loop();
		if (handsOfGodEnabled)
			handsOfGod.loop();
		telemetry.update();
	}

	private double waitTime = 0;
	private double startTime = 0;

	private double handMoveSeconds = .6;
	private double palmMoveSeconds = .5;

	private boolean justShot = false;

	/**
	 * Shoot a ball at an arbritrary position, will shoot ball in hands position if it is blocking the path
	 * returns true if it needs to run again
	 *
	 * @param position
	 * @return
	 */
	public boolean shoot(BallPosition position) {
		telemetry.addLine("rt: " + opMode.getRuntime() + " st " + startTime + " wt " + waitTime);
		if (opMode.getRuntime() - startTime < waitTime) return true;

		// If we are not shooting the hand ball but it is in the way we shoot the hand ball instead
		if (position != BallPosition.Hands) {
			if (handBall != None) {
				shoot(BallPosition.Hands);
			}
		}

		// If there is no ball in that position there is no reason to shoot
//		if (position.ball == None) {
//			return false;
//		}

		if (position == BallPosition.Hands) {
			if (handsOfGod.getPosition() != HandsOfGod.Position.Up && !justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Up);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = true;
				return true;
			}
			if (justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Down);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = false;
				handBall = None;
				return false;
			}
			return false;
		}

		if (position == BallPosition.Right) {
			// is this palm down
			if (palmsOfGod.getPalm(PalmsOfGod.Palm.Right) == PalmsOfGod.Position.Down) {
				waitTime = palmMoveSeconds;
				startTime = opMode.getRuntime();
				palmsOfGod.setRightPalm(PalmsOfGod.Position.Up);
				return true;
			}

			if (handsOfGod.getPosition() != HandsOfGod.Position.Up && !justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Up);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = true;
				return true;
			}
			if (justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Down);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = false;
				rightBall = None;
				return false;
			}
			return false;
		}

		if (position == BallPosition.Left) {
			// is this palm down
			if (palmsOfGod.getPalm(PalmsOfGod.Palm.Left) == PalmsOfGod.Position.Down) {
				waitTime = palmMoveSeconds;
				startTime = opMode.getRuntime();
				palmsOfGod.setLeftPalm(PalmsOfGod.Position.Up);
				return true;
			}

			if (handsOfGod.getPosition() != HandsOfGod.Position.Up && !justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Up);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = true;
				return true;
			}
			if (justShot) {
				handsOfGod.setPosition(HandsOfGod.Position.Down);
				waitTime = handMoveSeconds;
				startTime = opMode.getRuntime();
				justShot = false;
				leftBall = None;
				return false;
			}
			return false;
		}

		return false;
	}

	/**
	 * Hands, Right, Left
	 *
	 * @param balls
	 */
	public void setBalls(Ball... balls) {
		System.arraycopy(balls, 0, Robot.balls, 0, 3);
	}

	// <3 enums as lookup tables... sure wish there were macros in java for compile time existence checking
	/**
	 * List of robot parts.
	 * use Part.name when getting a part, so if a configuration is changed, this can be updated simply.
	 * Would recommend importing this as static
	 */
	@Configurable
	public enum Part {
		//Notice how doc can be added to variants to make them more understandable, please use this liberally, but not excessively
		/**
		 * Front Right drive motor
		 */
		MotorFR 	("motorFR", DcMotor.class),
		/**
		 * Front Left drive motor
		 */
		MotorFL 	("motorFL", DcMotor.class),
		/**
		 * Back Right drive motor
		 */
		MotorBR 	("motorBR", DcMotor.class),
		/**
		 * Back Left drive motor
		 */
		MotorBL 	("motorBL", DcMotor.class),
		/**
		 * Intake Motor
		 */
		Intake 		("intake", DcMotor.class),
		/**
		 * Encoder that measures strafe on the robot
		 */
		StrafeEncoder 		("motorBR", DcMotor.class),
		/**
		 * Encoder that measures drive on the left side of the robot
		 */
		LeftDriveEncoder 	("motorBL", DcMotor.class),
		/**
		 * Encoder that measures drive on the right side of the robot
		 */
		RightDriveEncoder 	("intake", DcMotor.class),
		LeftPalm	("leftPalm", Servo.class),
		RightPalm	("rightPalm", Servo.class),
		LeftHand	("leftHand", Servo.class),
		RightHand	("rightHand", Servo.class),
		/**
		 * Motor for accelerating balls
		 */
		LaunchMotor	("spinny0", DcMotor.class),
		/**
		 * Motor for rotating the turret
		 */
		TurretRotator	("turretRotator", DcMotor.class),
		/**
		 * Servo to lift the intake, and hopefully untake balls from the center
		 */
		IntakeLift	("intakeLift", Servo.class),
		LimeLight	("limelight", Limelight3A.class);
		//TODO! make this list exhaustive

		public final String name;
		public final Class<? extends HardwareDevice> type;

		Part(String name, Class<? extends HardwareDevice> type) {
			this.name = name;
			this.type = type;
		};

		//Note that methods can also be created
		public static List<Part> Servos() {
			return Stream.of(Part.values()).filter(part -> {return part.type == Servo.class;}).collect(Collectors.toList());
		}

		// Don't know when you would use this, its more of an example
		public static List<Part> driveMotors() {
			return Arrays.asList(new Part[]{MotorFR, MotorFL, MotorBR, MotorBL});
		}

		/**
		 * Returns true if the hardware device exists in the Driver Hub configuration, a return of true means that it is safe to call hardwaremap.get() on it
		 * @param hardwareMap
		 * @return Devices existence
		 */
		public boolean exists(HardwareMap hardwareMap) {
			//Kinda silly way to do this, if only errors as values were a thing
			try {
				hardwareMap.get(this.name);
				return true;
			} catch (Exception ignored) {
				return false;
			}
		}
	}
}

