package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "RobotTest")
public class RobotTest extends OpMode {
	Robot bot;
	MecanumDrive mecanumDrive;

	@Override
	public void init() {
		bot = new Robot(this).enableIntake().enableIntakeSpinner();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		bot.loop();

		bot.intakeSpinner.setPower(gamepad2.left_trigger);
		bot.intake.setSpeed(gamepad2.right_trigger);
		bot.intakeSpinner.setRotation(abs(gamepad2.right_stick_y));
	}
}
