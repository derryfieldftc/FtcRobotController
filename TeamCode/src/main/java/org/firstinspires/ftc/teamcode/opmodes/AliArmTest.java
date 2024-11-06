package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ArmBot;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;

@TeleOp(name = "Alis Op Mode", group = OpModeGroups.COMP)
public class AliArmTest extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		ArmBot armbot = new ArmBot(this);
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		return new RobotPlugin[] {
				armbot, mecanumDrive
		};
	}
}
