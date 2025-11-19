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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous()
@Configurable // Panels
public class Blue1 extends OpMode {

	private TelemetryManager panelsTelemetry; // Panels Telemetry instance
	public Follower follower; // Pedro Pathing follower instance
	private int pathState; // Current autonomous path state (state machine)
	private Paths paths; // Paths defined in the Paths class

	@Override
	public void init() {
		panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

		follower = Constants.createFollower(hardwareMap);
		follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

		paths = new Paths(follower); // Build paths

		panelsTelemetry.debug("Status", "Initialized");
		panelsTelemetry.update(telemetry);
	}

	@Override
	public void loop() {
		follower.update(); // Update Pedro Pathing
		pathState = autonomousPathUpdate(); // Update autonomous state machine

		// Log values to Panels and Driver Station
		panelsTelemetry.debug("Path State", pathState);
		panelsTelemetry.debug("X", follower.getPose().getX());
		panelsTelemetry.debug("Y", follower.getPose().getY());
		panelsTelemetry.debug("Heading", follower.getPose().getHeading());
		panelsTelemetry.update(telemetry);
	}

	public static class Paths {

		public PathChain CloseShot1;
		public PathChain Pickup2;
		public PathChain Lever4;
		public PathChain CloseShot4;
		public PathChain PickupBackRow5;
		public PathChain CloseShot6;
		public PathChain PickupFirstRow7;
		public PathChain CloseShot8;
		public PathChain Middle9;

		public Paths(Follower follower) {
			CloseShot1 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(23.487, 119.379), new Pose(51.996, 83.744))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			Pickup2 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(51.996, 83.744),
									new Pose(50.214, 56.693),
									new Pose(61.714, 58.637),
									new Pose(22.515, 59.285)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			Lever4 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(22.515, 59.285),
									new Pose(21.867, 67.384),
									new Pose(16.360, 65.926)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			CloseShot4 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(16.360, 65.926),
									new Pose(47.460, 60.256),
									new Pose(51.996, 84.067)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			PickupBackRow5 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(51.996, 84.067), new Pose(18.628, 83.906))
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			CloseShot6 = follower
					.pathBuilder()
					.addPath(
							new BezierLine(new Pose(18.628, 83.906), new Pose(54.749, 84.067))
					)
					.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
					.build();

			PickupFirstRow7 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(54.749, 84.067),
									new Pose(63.982, 32.720),
									new Pose(18.790, 35.312)
							)
					)
					.setTangentHeadingInterpolation()
					.build();

			CloseShot8 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(18.790, 35.312),
									new Pose(59.123, 30.938),
									new Pose(57.017, 83.906)
							)
					)
					.setConstantHeadingInterpolation(Math.toRadians(180))
					.build();

			Middle9 = follower
					.pathBuilder()
					.addPath(
							new BezierCurve(
									new Pose(57.017, 83.906),
									new Pose(57.179, 67.870),
									new Pose(32.882, 69.003)
							)
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
