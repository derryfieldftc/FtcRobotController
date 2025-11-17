package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.concurrent.Callable;

/**
 * Meant to replicate the way RR does actions
 */
public abstract class Action {
	/**
	 * This will be called until it returns false
	 *
	 * @return if it should be run again
	 */
	abstract boolean run();

	/**
	 * Takes a hold of the current thread and runs its action until it is finished, this method is not safe from force stops, and will **not** stop if the OpMode is told to stop. If that behavior is desired, call the actions yourself in the main loop.
	 */
	public static void runBlocking(Action action) {
		while (action.run()) {}
	}

	public static Action from(Runnable function) {
		return new Action() {
			@Override
			boolean run() {
				function.run();
				return false;
			}
		};
	}
}
