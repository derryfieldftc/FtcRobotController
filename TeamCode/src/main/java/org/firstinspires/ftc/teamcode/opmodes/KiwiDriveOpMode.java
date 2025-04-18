package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.KiwiBotArm;
import org.firstinspires.ftc.teamcode.plugins.OmniKiwiDrive;

@TeleOp(name="Kiwi Drive Test")
public class KiwiDriveOpMode extends PluginOpMode {
	public RobotPlugin[] initPlugins() {
		return new RobotPlugin[] { new OmniKiwiDrive(this), new KiwiBotArm(this)};
	}
}
