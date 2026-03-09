package org.firstinspires.ftc.teamcode.autonmous;

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
import org.firstinspires.ftc.teamcode.robot.Robot;

// This is our example auto function
@Autonomous // Similar to the ExampleOpMode we need to annotate with @Autonomous
public class ExampleAuto extends OpMode {
	// We need a robot and a follower for basic usage
    Robot robot;
	Follower follower;

	// And we also need an action to actually do everything
	boolean isRunning;
	Action action;

    @Override
    public void init() {
		// In our init() function we actually create our Robot, Follower, and Action
		robot = new Robot(this);
		follower = Constants.createFollower(hardwareMap);

		// When we create the follower we also need to pass an initial position
		follower.setStartingPose(new Pose(47, 75.86, 0));

		// We also do any other misc things that happen at the start of an OpMode
		telemetry.addLine("Init!");

		// We also make our Paths class
		Paths paths = new Paths(follower);

		isRunning = true;

		// Now is the interesting thing: The action
		// The action is the real meat and potatoes of your auto. It is created here in the init phase
		action = // While not necessary I like using a newline for the first action, as it makes the "layers" more obvious
				new ParallelAction( // Think of actions as layers. This parallel action runs each of the actions within it
						new SequentialAction( // The first action run in the parallel action is this sequential action, which runs each action in order until it has completed
								new FollowPathAction(follower, paths.Curve1), // This FollowPathAction makes the robot follow a pedropathing path, and finises once the robot completes the path
								new SleepAction(1000), // After the robot has followed the path, the sequential action sleeps for one second **Note that when the sequential action is sleeping, the other actions in the parallel action still get run
								new FollowPathAction(follower, paths.Straight2), // Then the robot follows another path, same way as the first
								robot.doSomethingAction(), // After we finish the previous action we call robot.doSomethingAction() which presumably does something
								new FollowPathAction(follower, paths.Wiggle3), // then we follow another path
								new FollowPathAction(follower, paths.Straight4) // And to finish its one more path
						),

						// These next parts are called every time run is called on the ParallelAction.
						new InstantAction(() -> robot.savePosition(follower)), // We save the robots position
						new InstantAction(() -> telemetry.update()) // and this is a weird one but we update the telemetry, this is weird because normally we just leave that in the loop. It does not reallllyyyy matter
				);

		// We only want to call telemetry.update() once per loop, so by convention it is never called in functions, and is called once at the end of every loop or init phase
		telemetry.update();
    }

	@Override
	public void loop() {
		if (isRunning)
			isRunning = action.run();
	}

	// This is the paths class. It has been pasted in from the pedropathing [visualizer](https://visualizer.pedropathing.com/)
	public static class Paths {
		public PathChain Curve1; // Notice how we use descriptive names followed by a number. If anything these names should describe more like where we are going rather than the shape but you get the idea
		public PathChain Straight2;
		public PathChain Wiggle3;
		public PathChain Straight4;

		// This is a constructor for the Paths object, and what we will use to create all the paths
		public Paths(Follower follower) {
			// While you will almost always use the visualizer to create paths, it is important to remember how to edit them yourself, rtfm
			Curve1 = follower.pathBuilder().addPath(
					new BezierCurve(
						new Pose(47.540, 75.862),
						new Pose(70.966, 28.747),
						new Pose(94.713, 77.609)
						)
					).setTangentHeadingInterpolation() // The main thing you might need to change manually with paths is the heading, so this doc includes multiple examples. Tangential means that the robot aims the same direction it moves

				.build();

			Straight2 = follower.pathBuilder().addPath(
					new BezierLine(
						new Pose(94.713, 77.609),

						new Pose(94.897, 99.126)
						)
					).setConstantHeadingInterpolation(Math.toRadians(90)) // Constant heading means that the robot faces the same direction the whole time it travels

				.build();

			Wiggle3 = follower.pathBuilder().addPath(
					new BezierCurve(
						new Pose(94.897, 99.126),
						new Pose(83.115, 78.891),
						new Pose(58.069, 122.247),
						new Pose(47.517, 99.644)
						)
					).setConstantHeadingInterpolation(Math.toRadians(90))

				.build();

			Straight4 = follower.pathBuilder().addPath(
					new BezierLine(
						new Pose(47.517, 99.644),

						new Pose(47.782, 75.885)
						)
					).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-90)) // Linear means it starts facing the first angle, and then rotates during the path to the second one

				.build();
		}
	}
}
