package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotTask;

public class DelayTask extends RobotTask {

	LinearOpMode opMode;
	long waitMillis;

	public DelayTask(LinearOpMode opMode) {
		this.opMode = opMode;
	}

	public DelayTask waitMillis(long waitMillis) {
		this.waitMillis = waitMillis;
		return this;
	}

	public void run() {
		opMode.sleep(waitMillis);
	}
}
