package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.MotorTest;
import org.firstinspires.ftc.teamcode.plugins.ServoTest;

@TeleOp(name = "theofores", group = OpModeGroups.SAMPLES)
public class TheodoresArm extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		MotorTest motorTest = new MotorTest(this)
				.addMotor("lift");
		/*
		ServoTest servoTest = new ServoTest(this)
				.addServo("angleH")
				.addServo("rotate")
				.addServo("claw");

		 */
		return new RobotPlugin[] { motorTest };
	}
}
