package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * This class is meant to be a container for many smaller subsystems. Each subsystem of the robot should focus on doing only one thing.
 * This robot class is meant to allow for a higher-up object, and is where we would do things like checking that a lift is out of the way.
 */
public class Robot extends RobotPart {

	// Make sure all subsystems are public
	public RobotVoltageSensor voltage;

	// This a weird subsection of the robot, being the follower. This is made by and for pedropathing.
	public Follower follower;

	public Robot(OpMode opMode) {
		// Because we extend RobotPart and call super() at the start, we do not need our own local OpMode or HardwareMap references
        super(opMode);

		// At this point also instantiate any sub-systems of the robot.
		// RobotVoltageSensor is used here as an example
		voltage = new RobotVoltageSensor(opMode);
	}

	/**
	 * This should be called once per loop, to update all parts of the robot
	 */
	// This function is weird. Not because it has interesting things going on in code, but because the idea behind it is weird.
	// In a program such as this, we should actually avoid large full-state updates like this. This is because they generally add
	// Extra unnecessary time to loops, and generally your program should be event based.
	public void update() {}

	// This is an example function
	public void HelloWorld() {
		telemetry.addLine("Hello World");
		telemetry.update();
	}

	double lastTime = 0;
	/**
	 * This function is used to check loop times. It returns the difference, in seconds, of the current time since the last time it was called.
	 * On its first call it will return garbage data.
	 */
	public double getTimeSinceLastUpdate() {
		double timeDiff = lastTime - opMode.getRuntime();
		lastTime = opMode.getRuntime();
		return timeDiff;
	}
}
