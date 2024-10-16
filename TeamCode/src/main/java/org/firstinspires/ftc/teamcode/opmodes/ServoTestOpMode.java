package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.ServoTest;

@TeleOp(name = "Servo Test", group = OpModeGroups.TESTS)
public class ServoTestOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		ServoTest servoTest = new ServoTest(this)
				.addServo("base")
				.addServo("shoulder")
				.addServo("elbow")
				.addServo("bend")
				.addServo("wrist")
				.addServo("claw");
		return new RobotPlugin[] { servoTest };
	}
}