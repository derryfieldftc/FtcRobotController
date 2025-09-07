package org.firstinspires.ftc.teamcode.plugin.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugin.plugins.MecanumDrive;


@TeleOp(name = "Mecanum Drive", group = OpModeGroups.SAMPLES)
public class MecanumDriveOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		return new RobotPlugin[] { mecanumDrive };
	}
}
