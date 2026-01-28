package org.firstinspires.ftc.teamcode.opmodes;

import android.icu.lang.UProperty;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "RobotTest")
public class RobotTest extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;
	GamepadManager mgamepad;
	boolean liftUp = false;
	boolean shootRight, shootHands, shootLeft;
	boolean leftPalmOpen = false, rightPalmOpen = false;

	@Override
	public void init() {
		bot = new Robot(this);
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
//		bot.turret.useGamepad();


		mgamepad = new GamepadManager(gamepad2);
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		bot.loop();

		bot.intake.setSpeed(gamepad2.right_trigger * ((gamepad2.y) ? -1 : 1));

		if (mgamepad.justPressed(GamepadManager.Button.X)) {
			liftUp = !liftUp;
		}

		bot.turret.setSpeed(-gamepad2.left_stick_y);

		bot.lift.setPosition(liftUp ? Lift.Position.Up : Lift.Position.Down);

		telemetry.update();
		mgamepad.poll();
	}
}
