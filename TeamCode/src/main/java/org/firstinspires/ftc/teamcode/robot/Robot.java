package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;
import static org.firstinspires.ftc.teamcode.robot.Field.motif;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 * If you run into any null pointers check your enabled parts of the robot
 */
public class Robot {
	private OpMode opMode;
	private HardwareMap hardwareMap;
	private Telemetry telemetry;
	public static Drivetrain drivetrain;
	public static boolean drivetrainEnabled;
	public static Intake intake;
	public static boolean intakeEnabled;
	//	public IntakeSpinner intakeSpinner;
//	public boolean intakeSpinnerEnabled;
	public static Camera camera;
	public static boolean cameraEnabled;
	public static Turret turret;
	public static boolean turretEnabled;
	public static HandsOfGod handsOfGod;
	public static boolean handsOfGodEnabled;
	public static PalmsOfGod palmsOfGod;
	public static boolean palmsOfGodEnabled;

	/**
	 * 1
	 * 3   2
	 */
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
		this.opMode = opMode;
		this.hardwareMap = opMode.hardwareMap;
		this.telemetry = opMode.telemetry;
		drivetrain = new Drivetrain(hardwareMap, this.opMode);
		intake = new Intake(this.opMode);
//		intakeSpinner = new IntakeSpinner(this.opMode);
		camera = new Camera(this.opMode);
		turret = new Turret(this.opMode, new TurretPose2d(new Pose2d(0, 0, 0), 0));
		handsOfGod = new HandsOfGod(this.opMode);
		palmsOfGod = new PalmsOfGod(this.opMode);
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

	public Robot enableCamera() {
		cameraEnabled = true;
		return this;
	}

	public void init() {
		if (intakeEnabled)
			intake.init();
//		if (intakeSpinnerEnabled)
//			intakeSpinner.init();
		if (cameraEnabled)
			camera.init();
		if (turretEnabled)
			turret.init();
		if (handsOfGodEnabled)
			handsOfGod.init();
		if (palmsOfGodEnabled)
			palmsOfGod.init();
	}

	public void loop() {
		if (intakeEnabled)
			intake.loop();
		if (cameraEnabled)
			camera.loop();
		if (turretEnabled)
			turret.loop();
		if (handsOfGodEnabled)
			handsOfGod.loop();
	}

	private double waitTime = 0;
	private double startTime = 0;

	private double handMoveSeconds = .5;
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
		telemetry.update();
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

	public Action shootAction(BallPosition position) {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				return shoot(position);
			}
		};
	}

	/**
	 * Shoot all balls whilst trying to match the motif
	 *
	 * @return
	 */
	public Action shootAllTryingMotif() {
		return new Action() {
			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {
				if (handBall != None) {
					return shoot(BallPosition.Hands);
				} else {
					if (motif.getBall(1) == rightBall) {
						return shoot(BallPosition.Right);
					} else {
						if (leftBall != None) {
							return shoot(BallPosition.Left);
						} else if (rightBall != None) {
							return shoot(BallPosition.Right);
						}
					}
				}
				return handBall != None || rightBall != None || leftBall != None;
			}
		};
	}

	/**
	 * Hands, Right, Left
	 *
	 * @param balls
	 */
	public void setBalls(Ball... balls) {
		System.arraycopy(balls, 0, Robot.balls, 0, 3);
	}

	public Action unloadBasedOnMotif() {
		return new Action() {
			Field.Motif motif = Field.motif;
			boolean unloading = true;

			@Override
			public boolean run(@NonNull TelemetryPacket telemetryPacket) {

				return unloading;
			}
		};
	}
}

