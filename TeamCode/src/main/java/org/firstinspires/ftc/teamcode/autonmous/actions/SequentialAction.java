package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

/**
 * Use this to run a list of actions, starting the next one after one finishes
 */
public class SequentialAction extends Action {
	ArrayList<Action> actions;
	int index = 0;
	boolean lastAction;

	/**
	 * A Sequential Action runs each of its arguments, and then advances to the next argument once one is finished
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
			RobotLog.d("AHM next sequential update");
			index += 1;
		}
		return index != actions.size();
	}

	/**
	 * Appends a new action to the end of the list
	 * @param action
	 */
	public void appendAction(Action action) {
		actions.add(action);
	}
}
