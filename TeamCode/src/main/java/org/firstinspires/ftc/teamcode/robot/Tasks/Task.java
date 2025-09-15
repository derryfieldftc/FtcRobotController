package org.firstinspires.ftc.teamcode.robot.Tasks;

public interface Task {
	/**
	 * What the string interpretation of the Task is, as it would be in a file containing a list of tasks
	 * @return
	 */
	String asString();

	/**
	 * Returns true if the string given is parseable by the task
	 * @param input
	 * @return
	 */
	boolean isParseable(String input);
	Task parse(String input);
	void execute();
	TaskType getType();
}
