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
import org.firstinspires.ftc.teamcode.autonmous.actions.InstantAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous()
@Configurable // Panels
public class TTBlue2Sorted extends OpMode {

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
		follower.setStartingPose(new Pose(48, 9, Math.toRadians(90)));

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
		robot.lift.setPosition(Lift.Position.Down);
		robot.spindexer.setLiftPosition(Spindexer.Height.Up);
		robot.spindexer.setBalls(Field.Ball.Green, Field.Ball.Purple, Field.Ball.Purple);
		action = new ParallelAction(
				new SequentialAction(
						new SleepAction(AutoConfigs.initalWaitTime),
						robot.shootAllSorted(),
						robot.spindexerPrepIntake(),
						robot.setIntakeSpeed(1),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						new FollowPathAction(follower, paths.Shoot2),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						// new SleepAction(AutoConfigs.postMovePreShootWait),
						robot.shootAllSorted(),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						robot.shootAllRemaining(),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake3),
						new FollowPathAction(follower, paths.Shoot4),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						robot.shootAllSorted(),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						robot.shootAllRemaining(),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake5),
						new FollowPathAction(follower, paths.Shoot6),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						new InstantAction(() -> robot.spindexer.updateBalls()),
						robot.shootAllSorted(),
						robot.spindexerPrepIntake(),

						new FollowPathAction(follower, paths.Middle7)),

				robot.turret.trackTarget(Depot.getPosition(Field.Alliance.Blue), follower.poseTracker.getLocalizer()),
				new Action() {
					@Override
					public boolean run() {
						robot.setTurretSpeed(robot.turret
								.getSpeedByDistance(robot.turret.getDistance(Depot.getPosition(Field.Alliance.Blue))))
								.run();
						return true;
					};
				},
				new Action() {
					@Override
					public boolean run() {
						robot.turret.savePosition();
						return true;
					}
				});
		while (robot.spindexer.resetPosition().run())
			;
	}

	@Override
	public void init_loop() {
		d("AHM searching");
		telemetry.addLine("Searching");
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
							new Pose(96.000, 9.000).mirror(),
							new Pose(96.389, 64.031).mirror(),
							new Pose(90.088, 52.332).mirror(),
							new Pose(125.557, 59.118).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

					.build();

			Shoot2 = follower.pathBuilder().addPath(
					new BezierCurve(
							new Pose(125.557, 59.118).mirror(),
							new Pose(101.584, 59.249).mirror(),
							new Pose(87.541, 17.439).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

					.build();

			Intake3 = follower.pathBuilder().addPath(
					new BezierCurve(
							new Pose(87.541, 17.439).mirror(),
							new Pose(96.949, 36.983).mirror(),
							new Pose(127.395, 35.519).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(164))

					.build();

			Shoot4 = follower.pathBuilder().addPath(
					new BezierLine(
							new Pose(127.395, 35.519).mirror(),

							new Pose(87.741, 17.678).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(164), Math.toRadians(180))

					.build();

			Intake5 = follower.pathBuilder().addPath(
					new BezierLine(
							new Pose(87.741, 17.678).mirror(),

							new Pose(132.030, 11.466).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(186))

					.build();

			Shoot6 = follower.pathBuilder().addPath(
					new BezierLine(
							new Pose(132.030, 11.466).mirror(),

							new Pose(84.283, 14.273).mirror()))
					.setLinearHeadingInterpolation(Math.toRadians(186), Math.toRadians(135))

					.build();

			Middle7 = follower.pathBuilder().addPath(
					new BezierLine(
							new Pose(84.283, 14.273).mirror(),

							new Pose(97.917, 28.063).mirror()))
					.setConstantHeadingInterpolation(Math.toRadians(135))

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
