package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ServoTest;

@TeleOp(name = "Servo Test", group = OpModeGroups.TESTS)
public class ServoTestOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		String[] servos = new String[] { "clawP", "rotate", "claw", "bucket" };
		ServoTest servoTest = new ServoTest(this, servos);
		return new RobotPlugin[] { servoTest };
	}
}