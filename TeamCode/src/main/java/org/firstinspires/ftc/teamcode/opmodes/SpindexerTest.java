package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotPart;
import org.firstinspires.ftc.teamcode.robot.Spindexer;

@TeleOp
public class SpindexerTest extends OpMode {
	Intake intake;
	Spindexer spindexer;
	Lift lift;
	DcMotor turret;

	@Override
	public void init() {
		spindexer = new Spindexer(this);
		spindexer.init();

		intake = new Intake(this);
		intake.init();

		lift = new Lift(this);
		lift.init();

		turret = hardwareMap.dcMotor.get(RobotPart.Part.LaunchMotor.name);
		turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	@Override
	public void loop() {
		intake.setSpeed(gamepad1.right_trigger);

		if (gamepad1.x) {
			spindexer.setPosition(Spindexer.Position.Zero);
		}
		if (gamepad1.a) {
			spindexer.setPosition(Spindexer.Position.One);
		}
		if (gamepad1.b) {
			spindexer.setPosition(Spindexer.Position.Two);
		}

		if (!gamepad1.y) {
			spindexer.setLiftPosition(Spindexer.Height.Up);
		} else {
			spindexer.setLiftPosition(Spindexer.Height.Down);
		}

		spindexer.setRotatorPower(gamepad1.left_trigger);

		if (gamepad1.dpad_down) {
			lift.setPosition(Lift.Position.Down);
		}
		if (gamepad1.dpad_up) {
			lift.setPosition(Lift.Position.Up);
		}

		turret.setPower(-(-gamepad1.right_stick_y + 1) / 2);
	}
}
