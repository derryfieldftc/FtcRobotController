package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 * If you run into any null pointers check your enabled parts of the robot
 */
public class Robot extends RobotPart {
	public Intake intake;
	public boolean intakeEnabled;
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

	/**
	 * This action sets a speed for the turret
	 * @param speed
	 * @return
	 */
	public Action setTurretSpeed(double speed) {
		return new Action() {
			@Override
			public boolean run() {
				turret.setSpeed(speed);
				return false;
			}
		};
	}

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
		intake = new Intake(this.opMode);
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

	public Robot enableTurret() {
		turretEnabled = true;
		return this;
	}

	public Robot enableIntake() {
		intakeEnabled = true;
		return this;
	}

	public double getVoltage() {
		return voltageSensor.getVoltage();
	}

	public void init() {
		if (intakeEnabled)
			intake.init();
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
	 * Shoots all balls, with no respect to sorting.
	 * @return
	 */
	public Action shootAll() {
		return new SequentialAction(
				shoot(BallPosition.Hands),
				new Action() {
					@Override
					public boolean run() {
						palmsOfGod.setLeftPalm(PalmsOfGod.Position.Up);
						shoot(BallPosition.Hands);
						return false;
					}
				},
				new Action() {
					@Override
					public boolean run() {
						palmsOfGod.setRightPalm(PalmsOfGod.Position.Up);
						shoot(BallPosition.Hands);
						return false;
					}
				}
		);
	}

	/**
	 * Shoot a ball at an arbitrary position, will shoot ball in hands position if it is blocking the path
	 * returns true if it needs to run again
	 *
	 * @param position
	 * @return
	 */
	public Action shoot(BallPosition position) {
		return new Action() {
			@Override
			public boolean run() {
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
		};
	}

	public Action setIntakeSpeed(double speed) {
		return new Action() {
			@Override
			public boolean run() {
				intake.setSpeed(speed);
				return false;
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

}

