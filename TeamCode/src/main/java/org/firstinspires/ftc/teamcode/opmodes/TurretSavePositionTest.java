package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
public class TurretSavePositionTest extends OpMode {
	org.firstinspires.ftc.teamcode.RR.MecanumDrive rr;
	MecanumDrive mecanumDrive;
	Turret turret;

	@Override
	public void init() {
		try {
			rr = new org.firstinspires.ftc.teamcode.RR.MecanumDrive(hardwareMap, Turret.getSavedPosition().pose2d);
			mecanumDrive = new MecanumDrive(this);
			turret = new Turret(this, Turret.getSavedPosition()).setTarget(new Vector2d(0, 0));
			turret.init();
			mecanumDrive.init();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		turret.autoTracking(rr).run(null);
		telemetry.addData("pose", rr.localizer.getPose().toString());

		if (gamepad1.a) {
			turret.savePosition();
			telemetry.addData("saved position at", this.getRuntime());
		}

		telemetry.update();
	}
}
