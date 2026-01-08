package org.firstinspires.ftc.teamcode.autonmous;

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
import com.sun.source.doctree.DeprecatedTree;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@Autonomous(name = "Blue1", group = "Autonomous")
@Configurable // Panels
public class Blue1 extends OpMode {

	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	public Follower follower; // Pedro Pathing follower instance
	private int pathState; // Current autonomous path state (state machine)
	private Paths paths; // Paths defined in the Paths class
	boolean completed = true;
	Robot robot;
	Action action;
	Field.Alliance alliance = Field.Alliance.Blue;

	@Override
	public void init() {
		robot = new Robot(this)
				.setTurretPose(new TurretPose(new Pose(24, 121, Math.toRadians(180)), 0));

		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(24, 121, Math.toRadians(180)));

		paths = new Paths(follower); // Build paths

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
		robot.turret.setRotationPower(1);

		action = new ParallelAction(
				new SequentialAction(
						new FollowPathAction(follower, paths.FirstShot1),
						robot.setIntakeSpeed(1),
						new FollowPathAction(follower, paths.PickupMidRow2),
						new FollowPathAction(follower, paths.Lever3),
						new SleepAction(500),
						new FollowPathAction(follower, paths.BackSecondShot4),
						new FollowPathAction(follower, paths.PickupBackRow5),
						new FollowPathAction(follower, paths.ThirdShot6),
						new FollowPathAction(follower, paths.PickUpClose7),
						new FollowPathAction(follower, paths.LastShot8)
				),
				robot.turret.trackTarget(Depot.getPosition(alliance), follower.getPoseTracker()
						.getLocalizer()),
				new Action() {
					@Override
					public boolean run() {
						robot.turret.setSpeed(robot.turret.getSpeedByDistance(robot.turret.getDistance(Depot.getPosition(alliance))));
						return true;
					}
	},
				new Action() {
					@Override
					public boolean run() {
						robot.turret.savePosition();
						return true;
					}
				}
		);
	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		pathState = autonomousPathUpdate(); // Update autonomous state machine
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

		public PathChain FirstShot1;
		public PathChain PickupMidRow2;
		public PathChain Lever3;
		public PathChain BackSecondShot4;
		public PathChain PickupBackRow5;
		public PathChain ThirdShot6;
		public PathChain PickUpClose7;
		public PathChain LastShot8;

		public Paths(Follower follower) {
			FirstShot1 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(24.000, 121.000), new Pose(63.010, 79.694))
					)
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
					.build();

			PickupMidRow2 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(63.010, 79.694),
									new Pose(60.094, 50.538),
									new Pose(18.790, 59.285)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			Lever3 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(18.790, 59.285),
									new Pose(22.029, 72.081),
									new Pose(10.529, 70.461)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			BackSecondShot4 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(10.529, 70.461),
									new Pose(49.566, 63.172),
									new Pose(55.559, 79.046)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			PickupBackRow5 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(55.559, 79.046),
									new Pose(45.516, 84.553),
									new Pose(19.276, 83.744)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			ThirdShot6 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(19.276, 83.744), new Pose(60.418, 75.645))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			PickUpClose7 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(60.418, 75.645),
									new Pose(72.081, 29.480),
									new Pose(16.036, 35.960)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			LastShot8 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(16.036, 35.960), new Pose(59.771, 75.159))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
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
