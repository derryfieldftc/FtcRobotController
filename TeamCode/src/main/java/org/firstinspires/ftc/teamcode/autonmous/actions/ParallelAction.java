package org.firstinspires.ftc.teamcode.autonmous.actions;

import org.firstinspires.ftc.teamcode.autonmous.AutoOpMode;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class ParallelAction extends Action {
	private final AutoOpMode autoOpMode;
	List<Action> actions;

	public ParallelAction(AutoOpMode autoOpMode, Action... actions) {
		this.autoOpMode = autoOpMode;
		this.actions = Arrays.asList(actions);
	}

	@Override
	boolean run() {
		actions = actions.stream().filter(Action::run).collect(Collectors.toList());
		return !actions.isEmpty();
	}
}
