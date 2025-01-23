package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.binarybot.BinaryBot;
import org.firstinspires.ftc.teamcode.binarybot.Manipulator;

@Autonomous(name="park", group = "BinaryBot")
public class Park extends LinearOpMode {
	BinaryBot bot;
	@Override
	public void runOpMode() {
		bot = new BinaryBot(hardwareMap, this);
		bot.measuredStrafe(.5, 27);
		bot.manipulator.calibrate();
		bot.manipulator.slide.setTargetPosition(Manipulator.SLIDE_MID_POSITION);
		bot.manipulator.deploy();
	}
}
