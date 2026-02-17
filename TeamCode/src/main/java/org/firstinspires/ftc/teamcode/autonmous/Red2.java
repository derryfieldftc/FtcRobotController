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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.InstantAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.LimeLight;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.Tag;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous()
@Configurable // Panels
public class Red2 extends OpMode {

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
		robot.spindexer.setBalls(Field.Ball.Green, Field.Ball.Purple, Field.Ball.Purple);

		action = new SequentialAction(
				new SleepAction(AutoConfigs.initalWaitTime),
				robot.shootAllSorted(),
				new InstantAction(() -> robot.spindexer.setLiftPosition(Spindexer.Height.Down)),
				new InstantAction(() -> {
					robot.spindexer.setPosition(Spindexer.Position.Zero);
					robot.spindexer.setLiftPosition(Spindexer.Height.Down);
				}),
				robot.setIntakeSpeed(1),
				new FollowPathAction(follower, paths.Pickup1),
				robot.setIntakeSpeed(0),
				new InstantAction(() -> robot.spindexer.setLiftPosition(Spindexer.Height.Up)),
				new FollowPathAction(follower, paths.CloseShot3),
				robot.shootAll(),
				new InstantAction(() -> {
					robot.spindexer.setPosition(Spindexer.Position.Zero);
					robot.spindexer.setLiftPosition(Spindexer.Height.Down);
				}),
				robot.setIntakeSpeed(1),
				new FollowPathAction(follower, paths.PickUpBackRow4),
				robot.setIntakeSpeed(0),
				new FollowPathAction(follower, paths.CloseShot5),
				new InstantAction(() -> robot.spindexer.setLiftPosition(Spindexer.Height.Up)),
//						new FollowPathAction(follower, paths.CloseShot4),
				robot.shootAll(),
				new InstantAction(() -> {
					robot.spindexer.setPosition(Spindexer.Position.Zero);
					robot.spindexer.setLiftPosition(Spindexer.Height.Down);
				}),
				robot.setIntakeSpeed(1),
				new FollowPathAction(follower, paths.PickUpFrontRow6),
				new InstantAction(() -> robot.spindexer.setLiftPosition(Spindexer.Height.Up)),
				new FollowPathAction(follower, paths.FarShot7),
				robot.shootAll(),
				new FollowPathAction(follower, paths.Middle8))

				.andAlso(robot.turret.trackTarget(Depot.getPosition(Field.Alliance.Red), follower.poseTracker.getLocalizer()))
				.andAlso(
						// These lower ones CANNOT be InstantActions because they need to run continuesly
						new Action() {
							@Override
							public boolean run() {
								robot.setTurretSpeed(robot.turret.getSpeedByDistance(robot.turret.getDistance(Depot.getPosition(Field.Alliance.Red)))).run();
								return true;
							};
						})
				.andAlso(
						new Action() {
							@Override
							public boolean run() {
								robot.turret.savePosition();
								return true;
							}
						});

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
		robot.spindexer.setRotatorPower(Spindexer.SpindexerConfig.speed);
		panelsTelemetry.debug("Set Power to ", Spindexer.SpindexerConfig.speed);
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

		public PathChain Pickup1;
		public PathChain Lever2;
		public PathChain CloseShot3;
		public PathChain PickUpBackRow4;
		public PathChain CloseShot5;
		public PathChain PickUpFrontRow6;
		public PathChain FarShot7;
		public PathChain Middle8;

		public Paths(Follower follower) {
			Pickup1 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(48.000, 12.000).mirror(),
									new Pose(74.025, 73.863).mirror(),
									new Pose(42.601, 58.799).mirror(),
									new Pose(18.973, 60.094).mirror()
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
					.build();

			Lever2 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(23.973, 60.094).mirror(),
									new Pose(23.163, 68.355).mirror(),
									new Pose(14.036, 69.651).mirror()
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			CloseShot3 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(16.036, 69.651).mirror(),
									new Pose(49.404, 65.440).mirror(),
									new Pose(53.939, 77.264).mirror()
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			PickUpBackRow4 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(53.939, 77.264).mirror(),
									new Pose(51.186, 86.173).mirror(),
									new Pose(18.733, 84.067).mirror()
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			CloseShot5 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(20.733, 84.067).mirror(), new Pose(53.939, 77.264).mirror())
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			PickUpFrontRow6 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(53.939, 77.264).mirror(),
									new Pose(66.898, 25.917).mirror(),
									new Pose(18.247, 35.474).mirror()
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			FarShot7 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(18.247, 35.474).mirror(), new Pose(54.475, 21.543).mirror())
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
					.build();

			Middle8 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(58.475, 21.543).mirror(), new Pose(26.403, 66.736).mirror())
					)
					.setConstantHeadingInterpolation(Math.toRadians(0))
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
