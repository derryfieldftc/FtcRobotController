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
public class TTRed2Sorted extends OpMode {

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
						robot.shootAllSortedInitial(Robot.balls),
						robot.spindexerPrepIntake(),
						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake1),
						new FollowPathAction(follower, paths.Shoot2),
						robot.spindexerPrepShoot(),
						new Action() {
							@Override
							public boolean run() {
								robot.spindexer.updateBalls();
								return true;
							};
						},
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootWait),
						robot.setIntakeSpeed(0),
						robot.shootAllSorted(Robot.balls, 0),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake3),
						new FollowPathAction(follower, paths.Shoot4),
						robot.spindexerPrepShoot(),
						new Action() {
							@Override
							public boolean run() {
								robot.spindexer.updateBalls();
								return true;
							};
						},
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootWait),
						robot.setIntakeSpeed(0),
						robot.shootAllSorted(Robot.balls, 0),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake5),
						new FollowPathAction(follower, paths.Shoot6),
						robot.spindexerPrepShoot(),

						new Action() {
							@Override
							public boolean run() {
								robot.spindexer.updateBalls();
								return true;
							};
						},
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootWait),
						robot.setIntakeSpeed(0),
						robot.shootAllSorted(Robot.balls, 0),
						robot.spindexerPrepIntake(),

						new FollowPathAction(follower, paths.Middle7)
						),

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
		public PathChain Intake1;
		public PathChain Shoot2;
		public PathChain Intake3;
		public PathChain Shoot4;
		public PathChain Intake5;
		public PathChain Shoot6;
		public PathChain Middle7;

		public Paths(Follower follower) {
			Intake1 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(96.000, 9.000),
									new Pose(96.389, 64.031),
									new Pose(90.088, 52.332),
									new Pose(125.557, 59.118)
							)
					).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

					.build();

			Shoot2 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(125.557, 59.118),
									new Pose(103.691, 59.249),
									new Pose(87.541, 17.439)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Intake3 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(87.541, 17.439),
									new Pose(96.949, 36.983),
									new Pose(130.971, 36.234)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(16))

					.build();

			Shoot4 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(130.971, 36.234),

									new Pose(87.741, 17.678)
							)
					).setLinearHeadingInterpolation(Math.toRadians(16), Math.toRadians(0))

					.build();

			Intake5 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(87.741, 17.678),

									new Pose(134.176, 10.751)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-6))

					.build();

			Shoot6 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(134.176, 10.751),

									new Pose(84.283, 14.273)
							)
					).setLinearHeadingInterpolation(Math.toRadians(-6), Math.toRadians(45))

					.build();

			Middle7 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(84.283, 14.273),

									new Pose(97.917, 28.063)
							)
					).setConstantHeadingInterpolation(Math.toRadians(45))

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
