package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Field.Ball;
import static org.firstinspires.ftc.teamcode.robot.Field.Ball.None;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;

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

	public Action setTurretSpeed_Full(double speed) {
		return new Action() {
			@Override
			public boolean run() {
				setTurretSpeed(speed).run();
				return true;
			}
		};
	}

	/**
	 * Do not forget to chain this with all of the enable methods
	 *
	 * @param opMode
	 */
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

	public Action getMotif(Localizer localizer) {
		return new Action() {
			@Override
			public boolean run() {
				turret.trackTarget(Obelisk.getObeliskPosition(), localizer);
				LLResult results = limeLight.getResults();

				if (results.isValid()) {
					for (LLResultTypes.FiducialResult tag : results.getFiducialResults()) {

						switch (tag.getFiducialId()) {
							case Tag.PGP.id:
								Field.motif = Obelisk.Motif.PGP;
							case Tag.PPG.id:
								Field.motif = Obelisk.Motif.PPG;
							case Tag.GPP.id:
								Field.motif = Obelisk.Motif.GPP;
							default:
								return true;
						}
					}
				}
			}
		};
	}
}

