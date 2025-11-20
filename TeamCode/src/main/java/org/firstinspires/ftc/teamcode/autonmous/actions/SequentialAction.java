package org.firstinspires.ftc.teamcode.autonmous.actions;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

public class SequentialAction extends Action {
	ArrayList<Action> actions;
	int index = 0;
	boolean lastAction;

	public SequentialAction(Action... actions) {
		this.actions = new ArrayList<>(Arrays.asList(actions));
		RobotLog.d("AHM", "SLGjfdljg", Arrays.toString(actions));
	}

	@Override
	public boolean run() {
		RobotLog.d("AHM %d", index);
		if (actions.isEmpty()) {
			return false;
		}
		if (!actions.get(index).run()) {
			RobotLog.d("AHM next sequential update");
			index += 1;
		}
		return index != actions.size();
	}
}
