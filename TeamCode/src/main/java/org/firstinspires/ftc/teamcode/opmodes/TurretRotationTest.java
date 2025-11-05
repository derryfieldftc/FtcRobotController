package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp
public class TurretRotationTest extends OpMode {
	Turret turret;
	MecanumDrive mecanumDrive;
	@Override
	public void init() {
		try {
			mecanumDrive = new MecanumDrive(hardwareMap, Turret.getSavedPosition().pose2d);
			turret = new Turret(this, Turret.getSavedPosition()).setTarget(new Vector2d(0, 1));
			turret.init();
		} catch (Exception e) {
			throw new RuntimeException(e);
		}

	}

	@Override
	public void loop() {
		turret.autoTracking(mecanumDrive).run(null);
		mecanumDrive.updatePoseEstimate();

		telemetry.addData("pos", turret.getRotation());
		telemetry.addData("as angle", turret.getRotation() / 2*Math.PI * 360);
		telemetry.update();
	}
}
