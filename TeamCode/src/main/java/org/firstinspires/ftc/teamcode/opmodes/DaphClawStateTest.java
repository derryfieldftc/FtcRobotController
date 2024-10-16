package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.DaphClaw;

@TeleOp(name="DaphClawStateTest", group = OpModeGroups.TESTS)
public class DaphClawStateTest extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		DaphClaw daphClaw = new DaphClaw(this);
		return new RobotPlugin[] { daphClaw };
	}
}
