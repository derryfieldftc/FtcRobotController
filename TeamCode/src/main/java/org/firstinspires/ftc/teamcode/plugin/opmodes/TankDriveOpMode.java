package org.firstinspires.ftc.teamcode.plugin.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.plugin.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugin.plugins.TankDrive;

@TeleOp(name = "TankDriveOpMode", group = OpModeGroups.SAMPLES)
public class TankDriveOpMode extends PluginOpMode {
    @Override
    public RobotPlugin[] initPlugins() {
        TankDrive tankDrive = new TankDrive(this);
        return new RobotPlugin[]{ tankDrive };
    }
}
