package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ITDclaw;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name = "CompOpMode", group = OpModeGroups.COMP)
public class CompOpMode extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		ITDclaw itDclaw = new ITDclaw(this);
		return new RobotPlugin[] { mecanumDrive, itDclaw };
	}
}
