package org.firstinspires.ftc.teamcode.autonmous.actions;

import org.firstinspires.ftc.teamcode.autonmous.AutoOpMode;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * This action concatenates actions into one that runs each child every time it is called. Once one of the children processes returns false it will stop being run
 */
public class ParallelAction extends Action {
	List<Action> actions;

	public ParallelAction(Action... actions) {
		this.actions = Arrays.asList(actions);
	}

	@Override
	public boolean run() {
		actions = actions.stream().filter(Action::run).collect(Collectors.toList());
		return !actions.isEmpty();
	}
}
