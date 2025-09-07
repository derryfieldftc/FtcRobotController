package org.firstinspires.ftc.teamcode.plugin.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.plugin.RobotTask;

public abstract class TaskAutonomous extends LinearOpMode {

	protected RobotTask[] tasks;
	public abstract RobotTask[] createTasksList();

	@Override
	public void runOpMode() {
		this.tasks = createTasksList();

		for (RobotTask task : tasks) { task.init(); }
		while (opModeInInit()) {
			for (RobotTask task : tasks) { task.init_loop(); }
		}

		waitForStart();

		for (RobotTask task : tasks) { task.run(); }
		for (RobotTask task : tasks) { task.stop(); }
	}
}
