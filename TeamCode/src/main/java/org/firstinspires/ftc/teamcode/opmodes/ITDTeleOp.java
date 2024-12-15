package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ITDClaw;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name = "CompOpMode", group = OpModeGroups.COMP)
public class ITDTeleOp extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		MecanumDrive drive = new MecanumDrive(this);
		ITDClaw claw = new ITDClaw(this);
		return new RobotPlugin[] { drive, claw };
	}
}
