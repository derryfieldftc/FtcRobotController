package org.firstinspires.ftc.teamcode.autonmous.actions;

/**
 * An action that sleeps for an amount of time, the timer starts the first time
 * run() is called.
 */
public class SleepAction extends Action {
	long startTime = -1;
	long sleepMs;

	/**
	 * 1000 ms = 1 s
	 * 
	 * @param sleepMs
	 */
	public SleepAction(long sleepMs) {
		this.sleepMs = sleepMs;
	}

	@Override
	public boolean run() {
		if (startTime == -1)
			startTime = System.currentTimeMillis();

		return System.currentTimeMillis() < startTime + sleepMs;
	}
}
