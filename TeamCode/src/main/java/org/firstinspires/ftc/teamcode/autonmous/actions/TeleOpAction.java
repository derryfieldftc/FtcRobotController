package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This is a funky action, because it is not an action in the normal sense, and it does not extend actions, if it should be started is passed to the run method
 */
public class TeleOpAction {
	Action action;
	boolean wasRunning = false;

	public TeleOpAction(Action action) {
		this.action = action;
	}

	/**
	 * Call this every loop, call it with start being true if you want to run the action, then it will continue running until finished, note that you cannot 'queue' or buffer actions after themselves by passing true multiple times, rather it will run once, regardless of how many times true is passed until it is done, at which case passing true makes it run again
	 * @param start
	 */
	public void run(boolean start) {
		try {
			if (start || wasRunning) {
				wasRunning = action.run();
			}
		} catch (Exception ignored) {}
	}
}
