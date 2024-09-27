package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotTask;

public class DelayTask extends RobotTask {
	OpMode opMode;
	long waitMillis;
	public DelayTask(OpMode opMode, long waitMillis) {
		this.opMode = opMode;
		this.waitMillis = waitMillis;
	}
	public void run() {
		double initalTime = opMode.getRuntime();
		while (opMode.getRuntime() < initalTime + waitMillis / 1000) {};
	}
}
