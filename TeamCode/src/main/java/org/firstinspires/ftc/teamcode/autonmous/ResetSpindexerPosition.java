package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Spindexer;

@Autonomous
public class ResetSpindexerPosition extends OpMode {
	Spindexer spindexer;

	@Override
	public void init() {
		spindexer = new Spindexer(this);
		spindexer.init();
	}

	@Override
	public void start() {
		super.start();
		while (spindexer.resetPosition().run());
	}

	@Override
	public void loop() {

	}
}
