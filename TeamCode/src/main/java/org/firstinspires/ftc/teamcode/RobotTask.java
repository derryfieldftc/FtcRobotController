package org.firstinspires.ftc.teamcode;

/**
 * This is the building blocks of every autonomous task
 * A task is one function that an autonomous OpMode performs
 */
public abstract class RobotTask {
	/**
	 * Code is run ONCE when the OPMODE starts
	 */
	public void init() {}

	/**
	 * Code is run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	public void init_loop() {}

	/**
	 * Code is ran each time RUN is called in the RobotTask array
	 * @return
	 */
	public void run() {}

	/**
	 * Code to run ONCE after the OPMODE stops
	 */
	public void stop() {}
}
