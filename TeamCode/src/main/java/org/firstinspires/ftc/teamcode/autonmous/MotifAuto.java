package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Motif Auto")
public class MotifAuto extends OpMode {
	boolean tagFound = false;
	Robot robot;

	@Override
	public void init() {
		Field.motif = null;
		robot = new Robot(this).enableCamera();
		robot.init();

	}

	@Override
	public void init_loop() {
		if (!tagFound) {
			if (robot.camera.getMotif().isPresent()) {
				Field.motif = robot.camera.getMotif().get();
				tagFound = true;
			}
		}
		telemetry.addData("tag found", tagFound);
		telemetry.update();
	}

	@Override
	public void loop() {
		telemetry.addData("motif", Field.motif);
		telemetry.update();
		robot.loop();
	}
}
