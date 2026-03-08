package org.firstinspires.ftc.teamcode.autonmous.actions;

/**
 * This is used to describe an action which will always be run until the stop()
 * method is called on it. We do this to standardize actions that don't finish,
 * while implementing this class, also remember to override the runForever()
 * method.
 */
public abstract class EternalAction extends Action {
	boolean running = true;

	/**
	 * Do not override this
	 */
	@Override
	final public boolean run() {
		this.runForever();
		return running;
	}

	/**
	 * This is where your actual function goes
	 */
	public abstract void runForever();

	/**
	 * Optional function that is called after stop() is called
	 */
	public void cleanup() {
	}

	final public void stop() {
		running = false;
		cleanup();
	}
}
