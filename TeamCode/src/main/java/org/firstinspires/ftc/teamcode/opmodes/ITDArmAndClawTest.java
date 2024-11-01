package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugins.ServoTest;

@TeleOp(name = "ITDClawTest", group = OpModeGroups.TESTS)
public class ITDArmAndClawTest extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		MecanumDrive mecanumDrive = new MecanumDrive(this);
		ServoTest servoTest = new ServoTest(this, "clawP", "rotate", "claw", "bucket");
		return new RobotPlugin[] { mecanumDrive, servoTest };
	}
}
