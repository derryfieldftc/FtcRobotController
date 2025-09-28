package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Camera;
import org.firstinspires.ftc.teamcode.robot.Tag;

@TeleOp(name = "AprilTagTest", group = OpModeGroups.TESTS)
@Disabled
public class AprilTagTestOpMode extends OpMode {
	MecanumDrive mecanumDrive;
	Camera camera;

	@Override
	public void init() {
		camera = new Camera(this).setTargetTag(Tag.PGP).setTagLocationTarget(320, 240);
		camera.init();
		hardwareMap.getClass(); // needed for setup somehow????
		mecanumDrive = new MecanumDrive(this);
		mecanumDrive.init();
	}

	@Override
	public void loop() {
		mecanumDrive.loop();
		camera.loop();
		camera.telemetry();
		telemetry.update();
	}
}