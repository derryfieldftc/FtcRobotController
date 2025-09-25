package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.robot.HandsOfGod;

@TeleOp(name = "HandsOfGodTest", group = OpModeGroups.TESTS)
@Disabled
public class HandsOfGodTest extends OpMode {
	HandsOfGod handsOfGod;

	@Override
	public void init() {
		handsOfGod = new HandsOfGod(this).useTelemetry()
				.useGamepad();
		handsOfGod.init();
	}

	@Override
	public void loop() {
		handsOfGod.loop();
		if (gamepad1.a) {
			handsOfGod.setPosition(HandsOfGod.Position.Up);
		}
		if (gamepad1.b) {
			handsOfGod.setPosition(HandsOfGod.Position.Down);
		}
		telemetry.update();
	}
}
