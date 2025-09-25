package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadManager;
import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;

@TeleOp(name = "IntakeTestOpMode", group = OpModeGroups.TESTS)
@Disabled
public class IntakeTestOpMode extends OpMode {
	RobotPlugin mechanumDrive;
	DcMotor intake;
	GamepadManager mgamepad;
	boolean intakeOn = true;
	boolean reverseDirection = false;

	@Override
	public void init() {
		mgamepad = new GamepadManager(gamepad1);
		hardwareMap.getClass(); //for setup
		mechanumDrive = new MecanumDrive(this);
		mechanumDrive.init();
		intake = hardwareMap.dcMotor.get("intake");
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}

	@Override
	public void loop() {
		mechanumDrive.loop();
		if (mgamepad.justPressed(GamepadManager.Button.A))
			intakeOn = !intakeOn;

		if (mgamepad.justPressed(GamepadManager.Button.B))
			reverseDirection = !reverseDirection;

		if (intakeOn) {
			intake.setPower(gamepad1.right_trigger * ((reverseDirection) ? -1 : 1));
		}


		telemetry.addData("intakePower", intake.getPower());
		telemetry.addData("intakeOn", intakeOn);
		mgamepad.poll();
	}
}
