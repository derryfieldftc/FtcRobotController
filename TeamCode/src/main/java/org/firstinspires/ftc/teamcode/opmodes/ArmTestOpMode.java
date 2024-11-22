package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ArmBot;
import org.firstinspires.ftc.teamcode.plugins.DeviceTest;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugins.prevArmBot;

@TeleOp(name="arm pos test", group = OpModeGroups.TESTS)
public class ArmTestOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		DeviceTest deviceTest = new DeviceTest(this, "base", "shoulder", "elbow", "wrist", "bend", "claw");
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		return new RobotPlugin[] { deviceTest, mecanumDrive };
	}
}
