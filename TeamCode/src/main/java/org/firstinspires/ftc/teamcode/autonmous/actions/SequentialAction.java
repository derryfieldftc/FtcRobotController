package org.firstinspires.ftc.teamcode.autonmous.actions;

import java.util.Arrays;
import java.util.Stack;

public class SequentialAction extends Action {
	Stack<Action> actions;

	public SequentialAction(Action... actions) {
		this.actions = new Stack<>();
		this.actions.addAll(Arrays.asList(actions));
	}

	@Override
	boolean run() {
		if (!actions.peek().run()) {
			actions.pop();
		}

		return !actions.empty();
	}
}
