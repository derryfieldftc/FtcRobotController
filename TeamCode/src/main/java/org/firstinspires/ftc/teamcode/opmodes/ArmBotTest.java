package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ArmBot;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugins.prevArmBot;

@TeleOp(name="ArmBotTest", group = OpModeGroups.TESTS)
public class ArmBotTest extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		prevArmBot armBot = new prevArmBot(this);
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		return new RobotPlugin[] { armBot, mecanumDrive };
	}
}
