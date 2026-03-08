# Alright so you want to use pedropathing

## Introduction
Great, its an amazing system to manage autonomous driving, and is even better through the use of actions.
Before you get into it there are a few notes
1. This is non-exhaustive, if you have a bigger question, then check the [docs](https://pedropathing.com/docs/pathing)
2. If that doesn't work, try asking someone for help.
3. If you don't have anyone or they don't know, use the FIRST community.

## Basic usage
So imagine you want to use a pedropathing path in your autonomous code. There are a few ways to do that.
The first and by far easiest is to use the [visualizer](https://visualizer.pedropathing.com/) This is how literally every path gets made. Use it.
To use it, first change the settings to match your robot dimensions. Then drag points and use the green + to create paths, you can also name paths, which is really helpful and strongly recommended, the current naming scheme is NameOfPathX, where X is the number of that path.
You can include sleep statements, but as of this update they don't do anything. Once you are happy with you path, use the export feature (top bar), and hit export to code, and then export as java code.
This gives you an object called Paths, and a bunch of members, being all the paths. (normally this code does not include comments, but those have been added for explanation)

```java
// This is the paths class
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
```
These paths are also used in the ExampleAuto program. Use that as a reference for a lot of this next bit.
Having these paths is great, but to actually use them you must create a follower, then set the path to follow, and update it, and keep track and a bunch of other things
so following has been mostly abstracted away into the FollowPathAction class. Again check the example auto for reference.


## Tuning
Tuning is an annoying but necessary process for any robot. This allows for the robot to have more accurate tracking.
Tuning needs to be done after any large changes to the robot, and, depending on how much you want is either incredibly annoying, or really simple.
The best guide for tuning is [this](https://pedropathing.com/docs/pathing/tuning) aka the docs. Just use them.
I will impart a little wisdom. The best way by far to tune the robot is with [Panels](https://panels.bylazar.com/) (Note that we technically use the Dairy version so it works with sloth, but docs *should* be the same, just not installation)

