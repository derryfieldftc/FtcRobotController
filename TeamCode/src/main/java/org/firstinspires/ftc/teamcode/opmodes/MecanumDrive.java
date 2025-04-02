package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.RobotPlugin;

public class MecanumDrive extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		return new RobotPlugin[] { new org.firstinspires.ftc.teamcode.plugins.MecanumDrive(this) };
	}
}
