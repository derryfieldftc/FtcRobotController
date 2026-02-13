package org.firstinspires.ftc.teamcode.autonmous;

import static com.qualcomm.robotcore.util.RobotLog.d;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous()
@Configurable // Panels
public class TTRed2 extends OpMode {

	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	public Follower follower; // Pedro Pathing follower instance
	private int pathState; // Current autonomous path state (state machine)
	private Paths paths; // Paths defined in the Paths class
	boolean completed = true;
	boolean resetting = true;
	Action action;
	Robot robot;

	@Override
	public void init() {
		robot = new Robot(this).setTurretPose(new TurretPose(new Pose(46, 9, Math.toRadians(135)).mirror(), 0));

		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(48, 9, Math.toRadians(90)).mirror());

		paths = new Paths(follower); // Build paths

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
		robot.turret.setRotationPower(1);

		d("AHM searching");
		telemetry.addLine("Searching");
		telemetry.update();
		robot.getMotif(follower.getPoseTracker().getLocalizer()).run();
		d("AHM FOUND " + Field.motif);
		telemetry.addData("motif", Field.motif);
		telemetry.update();

		robot.spindexer.setLiftPosition(Spindexer.Height.Up);

		action = new ParallelAction(
				new SequentialAction(
						new SleepAction(AutoConfigs.initalWaitTime),
						robot.shootAll(),
						robot.spindexerPrepIntake(),
						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.ToPickup1),

						new FollowPathAction(follower, paths.Intake2),
						new FollowPathAction(follower, paths.Shoot3),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(0),
						robot.shootAll(),
						robot.spindexerPrepIntake(),
						new FollowPathAction(follower, paths.Lever4),
						robot.setIntakeSpeed(1),
						new SleepAction(2000),
						robot.spindexerPrepShoot(),
						new FollowPathAction(follower, paths.Shoot5),
						robot.setIntakeSpeed(0),
						robot.shootAll(),
						robot.spindexerPrepIntake(),
						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake6),
						new FollowPathAction(follower, paths.Shoot7),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(0),
						robot.shootAll(),
						robot.spindexerPrepIntake(),
						new FollowPathAction(follower, paths.Middle8)),

				robot.turret.trackTarget(Depot.getPosition(Field.Alliance.Red), follower.poseTracker.getLocalizer()),
				new Action() {
					@Override
					public boolean run() {
						robot.setTurretSpeed(robot.turret.getSpeedByDistance(robot.turret.getDistance(Depot.getPosition(Field.Alliance.Red)))).run();
						return true;
					};
				},
				new Action() {
					@Override
					public boolean run() {
						robot.turret.savePosition();
						return true;
					}
				}
		);

		while (robot.spindexer.resetPosition().run());


	}

	@Override
	public void init_loop() {
		d("AHM searching");
		telemetry.addLine("Searching");
		telemetry.update();
		robot.getMotif(follower.getPoseTracker().getLocalizer()).run();
		d("AHM FOUND " + Field.motif);
		telemetry.addData("motif", Field.motif);
		telemetry.update();
	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		pathState = autonomousPathUpdate(); // Update autonomous state machine
		robot.spindexer.setRotatorPower(.9);
		if (completed)
			completed = action.run();

		panelsTelemetry.debug("action status", completed);

		// Log values to Panels and Driver Station
		panelsTelemetry.debug("Path State", pathState);
		panelsTelemetry.debug("X", follower.getPose().getX());
		panelsTelemetry.debug("Y", follower.getPose().getY());
		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
		panelsTelemetry.update(telemetry);
	}











	public static class Paths {
		public PathChain ToPickup1;
		public PathChain Intake2;
		public PathChain Shoot3;
		public PathChain Lever4;
		public PathChain Shoot5;
		public PathChain Intake6;
		public PathChain Shoot7;
		public PathChain Middle8;

		public Paths(Follower follower) {
			ToPickup1 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(96.000, 9.000),
									new Pose(91.987, 59.927),
									new Pose(102.278, 58.411)
							)
					).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

					.build();

			Intake2 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(102.278, 58.411),

									new Pose(128.550, 57.695)
							)
					).setTangentHeadingInterpolation()

					.build();

			Shoot3 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(128.550, 57.695),
									new Pose(93.232, 56.454),
									new Pose(96.020, 72.785),
									new Pose(84.179, 87.252)
							)
					).setConstantHeadingInterpolation(Math.toRadians(0))

					.build();

			Lever4 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(84.179, 87.252),
									new Pose(118.454, 47.513),
									new Pose(131.364, 61.748)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

					.build();

			Shoot5 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(131.364, 61.748),
									new Pose(126.358, 52.076),
									new Pose(83.695, 86.550)
							)
					).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

					.build();

			Intake6 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(83.695, 86.550),
									new Pose(109.126, 27.801),
									new Pose(97.781, 36.298),
									new Pose(126.742, 35.291)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Shoot7 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(126.742, 35.291),
									new Pose(102.126, 20.255),
									new Pose(86.675, 18.477)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Middle8 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(86.675, 18.477),

									new Pose(86.596, 28.291)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();
		}
	}










	public int autonomousPathUpdate() {
		// Add your state machine Here
		// Access paths with paths.pathName
		// Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
		return pathState;
	}
}
