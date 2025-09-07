package org.firstinspires.ftc.teamcode.plugin.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.plugin.RobotTask;

public class DelayTask extends RobotTask {

	OpMode opMode;
	long waitMillis;

	public DelayTask(OpMode opMode) {
		this.opMode = opMode;
	}

	public DelayTask waitMillis(long waitMillis) {
		this.waitMillis = waitMillis;
		return this;
	}

	public void run() {
		try {
			opMode.wait(waitMillis);
		} catch (Exception e) {
			opMode.telemetry.addLine("Exception while waiting: " + e);
		}
	}
}
