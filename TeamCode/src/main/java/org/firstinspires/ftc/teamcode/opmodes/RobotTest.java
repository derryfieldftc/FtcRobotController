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
		bot = new Robot(this).enableIntake();
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
		bot.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		bot.loop();

		bot.intake.setSpeed(gamepad2.right_trigger);
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
	}
}
