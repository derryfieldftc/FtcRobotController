package org.firstinspires.ftc.teamcode.autonmous.actions;

import java.util.function.Function;

/**
 * Meant to replicate the way RR does actions
 */
public abstract class Action {
	/**
	 * This should be called until it returns false
	 *
	 * @return if it should be run again
	 */
	public abstract boolean run();

	/**
	 * Takes a hold of the current thread and runs its action until it is finished, this method is not safe from force stops, and will **not** stop if the OpMode is told to stop. If that behavior is desired, call the actions yourself in the main loop.
	 */
	public static void runBlocking(Action action) {
		while (action.run()) {}
	}

	public static <T, R> Action from(Function<T, R> function, T input) {
		return new Action() {
			@Override
			public boolean run() {
				function.apply(input);
				return false;
			}
		};
	}

	/**
	 * Returns a new SequentialAction made of this and any other action
	 */
	public final SequentialAction andThen(Action nextAction) {
		return new SequentialAction(this, nextAction);
	};

	/**
	 * Returns a new ParallelAction made of this and any other action
	 * @param otherAction
	 * @return
	 */
	public final ParallelAction andAlso(Action otherAction) {
		return new ParallelAction(this, otherAction);
	}

}
