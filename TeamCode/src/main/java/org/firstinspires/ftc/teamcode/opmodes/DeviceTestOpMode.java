package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;

@TeleOp(name = "Motors Test", group = OpModeGroups.TESTS)
public class DeviceTestOpMode extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		DeviceTest deviceTest = new DeviceTest(this,
				"motorFR",
				"motorFL",
				"motorBR",
				"motorBL",
				"pitch",
				"claw",
				"clawP",
				"rotate",
				"bucket");


		return new RobotPlugin[] { deviceTest };
	}
}
