package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotTask;

public abstract class AutonomousOpMode extends LinearOpMode {

	public abstract RobotTask[] createTasksList();

	private RobotTask[] tasks;

	@Override
	public void runOpMode() {
		this.tasks = createTasksList();


		for (RobotTask task : tasks) {
			task.init();
		}

		while (opModeInInit()) {
			for (RobotTask task : tasks) {
				task.init_loop();
			}
		}

		waitForStart();

		for (RobotTask task : tasks) {
			task.run();
		}

		for (RobotTask task : tasks) {
			task.stop();
		}


	}
}
