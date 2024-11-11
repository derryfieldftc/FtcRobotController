package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.TankDrive;

@TeleOp(name = "TankDriveOpMode", group = OpModeGroups.SAMPLES)
public class TankDriveOpMode extends PluginOpMode {
	@Override
	public RobotPlugin[] initPlugins() {
		TankDrive tankDrive = new TankDrive(this);
		return new RobotPlugin[]{ tankDrive };
	}
}
