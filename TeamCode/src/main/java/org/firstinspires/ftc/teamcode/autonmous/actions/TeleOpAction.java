package org.firstinspires.ftc.teamcode.autonmous.actions;

import java.util.function.Supplier;

/**
 * This is a funky action, because it is not an action in the normal sense
 */
public class TeleOpAction extends Action {
	Action action;
	Supplier<Action> actionSupplier; // We use this supplier to get a new action each time ours finishes, because calling run on an action that has already been completed is undefined
	boolean wasRunning = false;

	public TeleOpAction(Supplier<Action> actionSupplier) {
		this.action = actionSupplier.get();
		this.actionSupplier = actionSupplier;
	}

	@Override
	public boolean run() {
		if (wasRunning) {
			wasRunning = action.run();
			if (!wasRunning) {
				action = actionSupplier.get();
			}
		}
		return wasRunning;
	}

	public void start() {
		wasRunning = true;
	}

	public void reset() {
		wasRunning = false;
		action = actionSupplier.get();
	}

	public boolean isRunning() {
		return wasRunning;
	}
}
