package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name="Motor and Servo Test", group = OpModeGroups.TESTS)
public class ITDMotorsAndServoTest extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		DeviceTest deviceTest = new DeviceTest(this, "shoulder", "claw", "bucket", "wrist", "tilt", "elbow", "slide");
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		return new RobotPlugin[] { deviceTest, mecanumDrive };
	}
}
