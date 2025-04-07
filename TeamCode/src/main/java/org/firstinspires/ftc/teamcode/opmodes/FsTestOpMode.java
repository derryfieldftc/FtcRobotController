package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.FSTest;

@TeleOp(name = "FsTest")
public class FsTestOpMode extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		return new RobotPlugin[] { new FSTest(this) };
	}
}
