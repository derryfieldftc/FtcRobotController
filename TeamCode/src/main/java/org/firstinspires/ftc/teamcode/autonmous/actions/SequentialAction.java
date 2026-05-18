package org.firstinspires.ftc.teamcode.autonmous.actions;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Use this to run a list of actions, starting the next one after one finishes
 */
public class SequentialAction extends Action {
	ArrayList<Action> actions;
	int index = 0;
	boolean lastAction;

	/**
	 * A Sequential Action runs each of its arguments, and then advances to the next
	 * argument once one is finished
	 */
	public SequentialAction(Action... actions) {
		this.actions = new ArrayList<>(Arrays.asList(actions));
	}

	@Override
	public boolean run() {
		if (actions.isEmpty() || index == actions.size()) {
			return false;
		}
		if (!actions.get(index).run()) {
			index += 1;
		}
		return index != actions.size();
	}

	/**
	 * Appends new actions to the end of the list
	 */
	public void appendActions(Action... actions) {
        this.actions.addAll(Arrays.asList(actions));
	}

	/**
	 * Returns the currently running action
	 * @return
	 */
	public Action getCurrentAction() {
		return actions.get(index);
	}

	/**
	 * Returns the action at the specified index
	 * @param index
	 * @return
	 * @throws IndexOutOfBoundsException
	 */
	public Action getAction(int index) throws IndexOutOfBoundsException {
		return actions.get(index);
	}
}
