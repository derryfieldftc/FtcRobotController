package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.MotorManagerTest;

@TeleOp(name="Motor Manager OpMode")
public class MotorManagerOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		return new RobotPlugin[] { new MotorManagerTest(this) };
	}
}
