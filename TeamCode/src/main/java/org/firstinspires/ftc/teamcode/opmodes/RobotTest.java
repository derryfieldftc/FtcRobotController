package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
<<<<<<< HEAD
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
=======
>>>>>>> f8dd7c239778a0eaa8f05a35e71a0d796e237ef5

import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;

<<<<<<< HEAD
@TeleOp(name = "RobotTest")
=======
>>>>>>> f8dd7c239778a0eaa8f05a35e71a0d796e237ef5
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
