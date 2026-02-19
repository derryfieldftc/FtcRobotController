package org.firstinspires.ftc.teamcode.autonmous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Field;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous
@Disabled
public class GetMotifTest extends OpMode {
	Robot robot;
	@Override
	public void init() {
		Follower follower = Constants.createFollower(this.hardwareMap);

		robot = new Robot(this);

		while (robot.getMotif(follower.getPoseTracker().getLocalizer()).run());
		telemetry.addData("", Field.motif);
		telemetry.update();
	}

	@Override
	public void loop() {

	}
}
