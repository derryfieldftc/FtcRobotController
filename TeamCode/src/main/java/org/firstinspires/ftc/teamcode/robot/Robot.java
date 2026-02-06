package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
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

	public Robot setTurretPose(TurretPose pose) {
		turret.setPose(pose);
		return this;
	}

	/**
	 * TODO! make work with color sensors
	 * @return
	 */
	public Action shootAll() {
		return new SequentialAction(
				new Action() {
					@Override
					public boolean run() {
						spindexer.setPosition(Spindexer.Position.Zero);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.setPosition(Spindexer.Position.One);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot(),
				new Action() {
					@Override
					public boolean run() {
						spindexer.setPosition(Spindexer.Position.Two);
						return false;
					}
				},
				spindexer.waitUntilFinished(1),
				this.shoot()
				);
	}

	@Configurable
	static class LiftTime {
		static long liftMillis = 600;
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
						return false;
					}
				},
				new SleepAction(LiftTime.liftMillis)
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

