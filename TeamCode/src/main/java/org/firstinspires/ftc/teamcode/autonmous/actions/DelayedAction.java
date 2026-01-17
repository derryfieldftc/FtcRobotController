package org.firstinspires.ftc.teamcode.autonmous.actions;

/**
 * Runs an action after waiting a specific amount of milliseconds
 */
public class DelayedAction extends Action {
	Action action;

	public DelayedAction(long delay, Action action) {
		this.action = new SequentialAction(new SleepAction(delay), action);

	}

	@Override
	public boolean run() {
		return action.run();
	}
}
