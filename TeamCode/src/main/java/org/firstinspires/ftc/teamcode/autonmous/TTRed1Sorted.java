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
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Spindexer;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous()
@Configurable // Panels
public class TTRed1Sorted extends OpMode {

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
		follower.setStartingPose(new Pose(112, 132, Math.toRadians(180)));

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
						new FollowPathAction(follower, paths.Shoot1),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						robot.shootAllSorted(),
						robot.spindexerPrepIntake(),
						robot.setIntakeSpeed(1),

						new FollowPathAction(follower, paths.Intake2),
						new FollowPathAction(follower, paths.Shoot3),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						robot.shootAll(),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake4),
						new FollowPathAction(follower, paths.Shoot5),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						robot.shootAll(),
						robot.spindexerPrepIntake(),

						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.Intake6),
						new FollowPathAction(follower, paths.Shoot7),
						robot.spindexerPrepShoot(),
						robot.setIntakeSpeed(-.5),
						new SleepAction(AutoConfigs.preShootReverseIntakeWait),
						robot.setIntakeSpeed(0),
						new SleepAction(AutoConfigs.postMovePreShootWait),
						robot.shootAll(),
						robot.spindexerPrepIntake(),

						new FollowPathAction(follower, paths.Middle8)
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
		public PathChain Shoot1;
		public PathChain Intake2;
		public PathChain Shoot3;
		public PathChain Intake4;
		public PathChain Shoot5;
		public PathChain Intake6;
		public PathChain Shoot7;
		public PathChain Middle8;

		public Paths(Follower follower) {
			Shoot1 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(111.000, 132.000),

									new Pose(85.801, 83.682)
							)
					).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))

					.build();

			Intake2 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(85.801, 83.682),

									new Pose(122.841, 83.404)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Shoot3 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(122.841, 83.404),

									new Pose(86.305, 79.391)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Intake4 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(86.305, 79.391),
									new Pose(100.222, 57.924),
									new Pose(121.238, 59.000)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

					.build();

			Shoot5 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(121.238, 59.000),
									new Pose(100.278, 57.460),
									new Pose(82.709, 80.901)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))

					.build();

			Intake6 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(82.709, 80.901),
									new Pose(90.447, 33.556),
									new Pose(123.497, 35.285)
							)
					).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))

					.build();

			Shoot7 = follower.pathBuilder().addPath(
							new BezierCurve(
									new Pose(123.497, 35.285),
									new Pose(98.904, 35.838),
									new Pose(88.212, 10.728)
							)
					).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

					.build();

			Middle8 = follower.pathBuilder().addPath(
							new BezierLine(
									new Pose(88.212, 10.728),

									new Pose(95.470, 36.788)
							)
					).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))

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
