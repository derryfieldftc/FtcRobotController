package org.firstinspires.ftc.teamcode.opmodes.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Depot;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.LimeLight;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.TurretPose;

@TeleOp
@Disabled
public class TurretAngleTest extends OpMode {
	Turret turret;
	LimeLight ll;
	MecanumDrive mecanumDrive;
	Follower drivetrain;

	@Override
	public void init() {
		mecanumDrive = new MecanumDrive(this);
		ll = new LimeLight(this);
		drivetrain = Constants.createFollower(this.hardwareMap);
		drivetrain.setStartingPose(new Pose(72, 72, 0));
		turret = new Turret(this, new TurretPose(new Pose(72, 72, 0), 0));
		mecanumDrive.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		drivetrain.update();
		turret.setRotationPower(.1);
		turret.trackTarget(Depot.getPosition(Field.Alliance.Red), drivetrain.getPoseTracker()
				.getLocalizer()).run();
		turret.setSpeed(turret.getSpeedByDistance(turret.getDistance(Depot.getPosition(Field.Alliance.Red))));
		telemetry.addData("pose", drivetrain.getPose().toString());
		telemetry.addData("DISTANCE", turret.getDistance(Depot.getPosition(Field.Alliance.Red)));
		telemetry.update();
	}
}
