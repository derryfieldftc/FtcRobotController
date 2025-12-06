package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "LogTest")
@Disabled
public class LogTest extends OpMode {
	ElapsedTime elapsedTime = new ElapsedTime();
	@Override
	public void init() {
		elapsedTime.reset();
		RobotLog.a(String.format("AHM initialized"));

	}

	@Override
	public void loop() {
		RobotLog.a(String.format("AHM elapsed time: %.3f", elapsedTime.milliseconds()));
	}
}
