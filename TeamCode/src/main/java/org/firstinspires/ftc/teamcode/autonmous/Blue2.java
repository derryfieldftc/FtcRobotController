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

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.PalmsOfGod;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous()
@Configurable // Panels
public class Blue2 extends OpMode {

	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	public Follower follower; // Pedro Pathing follower instance
	private int pathState; // Current autonomous path state (state machine)
	private Paths paths; // Paths defined in the Paths class
	Action execute, followPath, autoTrack;
	boolean following = true;
	Robot robot;

	@Override
	public void init() {
		robot = new Robot(this);

		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(48, 8, Math.toRadians(135)));

		paths = new Paths(follower); // Build paths

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
		followPath = new FollowPathAction(follower, paths.Pickup1, paths.Lever2, paths.CloseShot3, paths.PickUpBackRow4, paths.CloseShot5, paths.PickUpFrontRow6, paths.FarShot7, paths.Middle8);
	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		pathState = autonomousPathUpdate(); // Update autonomous state machine
		if (following)
			following = followPath.run();

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
									new Pose(48.000, 12.000),
									new Pose(74.025, 73.863),
									new Pose(42.601, 58.799),
									new Pose(23.973, 60.094)
							)
					)
					.setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
					.build();

			Lever2 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(23.973, 60.094),
									new Pose(23.163, 68.355),
									new Pose(16.036, 69.651)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			CloseShot3 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(16.036, 69.651),
									new Pose(49.404, 65.440),
									new Pose(53.939, 77.264)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.setReversed()
					.build();

			PickUpBackRow4 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(53.939, 77.264),
									new Pose(51.186, 86.173),
									new Pose(20.733, 84.067)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			CloseShot5 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(20.733, 84.067), new Pose(53.939, 77.264))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			PickUpFrontRow6 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(53.939, 77.264),
									new Pose(66.898, 25.917),
									new Pose(20.247, 35.474)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			FarShot7 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(20.247, 35.474), new Pose(58.475, 21.543))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			Middle8 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(58.475, 21.543), new Pose(26.403, 66.736))
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
