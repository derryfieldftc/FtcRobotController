package org.firstinspires.ftc.teamcode.robot;

import static com.qualcomm.robotcore.util.RobotLog.d;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
import org.firstinspires.ftc.teamcode.opmodes.Tests.ShootAllTest;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Class meant to easily hold all other robot classes, define positions and add methods as necessary
 * If you run into any null pointers check your enabled parts of the robot
 */
public class Robot extends RobotPart {
	public Intake intake;
	public Turret turret;
	public Spindexer spindexer;
	public Lift lift;
	public LimeLight limeLight;
	public TurretPose turretPose;


	/**
	 * 1
	 * 3   2
	 */
	// lol final = immutable yeah sure man
	public static final Ball[] balls = {None, None, None};
	public static Ball firstBall = balls[0];
	public static Ball secondBall = balls[1];
	public static Ball thirdBall = balls[2];

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

	// Returns only the X and Y of the robot
	public Pose getLLPose(Follower follower) {
		limeLight.setMode(LimeLight.LimeLightMode.Localization);
		Pose3D llpose = limeLight.ll.getLatestResult().getBotpose_MT2();

		return new Pose(llpose.getPosition().x * 3.28084, llpose.getPosition().y * 3.28084, follower.getHeading(), FTCCoordinates.INSTANCE);
	}

	public Robot setTurretPose(TurretPose pose) {
		turret.setPose(pose);
		return this;
	}

	public Action shootAllWithRegardForColor() {
		return new SequentialAction();
	}

	public Action shootAll() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Zero, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.One, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Two, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot()
				);
	}
	public Action shootAllFromPos(Spindexer.Position targetPosition, Lift lift) {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(targetPosition, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelyPreviousPosition(lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelyPreviousPosition(lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot()
		);
	}

	public Action shootAllOne() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.One, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Two, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Zero, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot()
		);
	}
	public Action shootAllTwo() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Two, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.Zero, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.safelySetPosition(Spindexer.Position.One, lift);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot()
		);
	}

	public Action shootAllSorted(Field.Ball[] balls) {
		if (Field.motif.getBall(0) == Field.Ball.Green) {
			d("TCT:Shooting All pos 0, green is first");
			return shootAllFromPos(Spindexer.Position.Zero, lift);
		} else if (Field.motif.getBall(1) == Field.Ball.Green) {
			d("TCT:Shooting All pos 2, green is second");
			return shootAllFromPos(Spindexer.Position.One, lift);
		} else {
			d("TCT:Shooting All pos 1, green is last");
			return shootAllFromPos(Spindexer.Position.Two, lift);
		}
	}
	@Configurable
	static class LiftTime {
		static long liftMillis = 350;
	}

	/**
	 * Shoots whatever ball is in a currently shootable position
	 * @return
	 */
	public Action shoot() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						lift.setPosition(Lift.Position.Up);
						return false;
					}
				},
				new SleepAction(LiftTime.liftMillis),
				new Action() {
					@Override
					public boolean run() {
						lift.setPosition(Lift.Position.Down);
						return !lift.isDown();
					}
				}
		);
	}

	public Robot(OpMode opMode) {
		super(opMode);
		intake = new Intake(this.opMode);
		turret = new Turret(this.opMode, new TurretPose(new Pose(0, 0, 0), 0));
		lift = new Lift(this.opMode);
		spindexer = new Spindexer(this.opMode);
		limeLight = new LimeLight(this.opMode);

		voltageSensor = hardwareMap.voltageSensor.iterator()
				.next(); // funky but also how RR gets voltage sensor
	}

	public double getVoltage() {
		return voltageSensor.getVoltage();
	}

	double currentTime = 0;

	public void loop() {
		currentTime = opMode.getRuntime();
		telemetry.update();
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
	public Action spindexerPrepIntake() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						if (lift.isDown()) {
							return false;

						} else {
							lift.setPosition(Lift.Position.Down);
							return true;
						}
					}
				},
				new Action() {
					@Override
					public boolean run() {
						spindexer.setPosition(Spindexer.Position.Zero);
						return false;
					}
				},
				new Action() {
					@Override
					public boolean run() {
						spindexer.setLiftPosition(Spindexer.Height.Down);
						return false;
					}
				}

		);
	}
	public Action spindexerPrepShoot() {
		return new Action() {
			@Override
			public boolean run() {
				spindexer.setLiftPosition(Spindexer.Height.Up);
				return false;
			}
		};

	}
	/**
	 * Updates Field.motif value, do not forget to correct the target after
	 * @param localizer
	 * @return
	 */
	public Action getMotif(Localizer localizer) {
		//TODO! make not aim towards obelisk cause its annoying
		return new Action() {
			@Override
			public boolean run() {
				turret.trackTarget(Obelisk.getObeliskPosition(), localizer);
				LLResult results = limeLight.getResults();
				limeLight.setMode(LimeLight.LimeLightMode.AprilTag);

				if (results.isValid()) {
					for (LLResultTypes.FiducialResult tag : results.getFiducialResults()) {

						switch (tag.getFiducialId()) {
							case 22: // Magic numbers corresponding to the ids of tags, can be found in the Tag class. Should work, but Enum variants are constructed at runtime, while switches require compile time information
								Field.motif = Obelisk.Motif.PGP;
								return false;
							case 23:
								Field.motif = Obelisk.Motif.PPG;
								return false;
							case 21:
								Field.motif = Obelisk.Motif.GPP;
								return false;
							default:
						}
					}
				}
				return true;
			}
		};
	}
}

