package org.firstinspires.ftc.teamcode.autonmous.actions;

//TODO! fix
public class LoopingAction extends Action {
	Action action;
	boolean runAgain;

	public LoopingAction(Action action, boolean runAgain) {
		this.action = action;
		this.runAgain = runAgain;
	}

	@Override
	public boolean run() {
		if (action.run()) {
			return true;
		} else {
			return runAgain;
		}
	}
}
