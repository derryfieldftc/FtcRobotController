package org.firstinspires.ftc.teamcode.autonmous.actions;

/**
 * This is useful when a action needs to be changed while running. It provides an easy way to replace its run() method, that being the setInnerAction() method
 */
public class WrappedAction extends Action {
    Action action;

    public WrappedAction(Action action) {
        this.action = action;
    }

    @Override
    public boolean run() {
        return action.run();
    }

    /**
     * Sets the inner action. This is used when changing the run() method
     * @param action
     */
    public void setInnerAction(Action action) {
        this.action = action;
    }
}
