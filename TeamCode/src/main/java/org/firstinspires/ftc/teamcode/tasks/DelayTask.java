package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotTask;

public class DelayTask extends RobotTask {
	OpMode opMode;
	public DelayTask(OpMode opMode) {
		this.opMode = opMode;
	}
	public RobotTask run(long waitMillis) {
		try {
			opMode.wait(waitMillis);
		} catch (Exception e) {
			opMode.telemetry.addLine("Exception while waiting: " + e);
		}
		return this;
	}
}
