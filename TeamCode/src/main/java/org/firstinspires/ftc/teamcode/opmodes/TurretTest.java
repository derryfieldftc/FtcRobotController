package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "TurretTest", group = OpModeGroups.TESTS)
@Disabled
public class TurretTest extends OpMode {
	MecanumDrive mecanumDrive;
	Turret turret;

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(this);
		turret = new Turret(this);

		mecanumDrive.init();
		turret.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		turret.loop();
	}
}
