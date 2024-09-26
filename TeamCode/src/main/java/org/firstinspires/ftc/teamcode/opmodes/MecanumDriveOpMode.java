package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.MecanumDrive;
import org.firstinspires.ftc.teamcode.plugins.TankDrive;

@TeleOp(name = "MecanumDriveOpMode", group = OpModeGroups.SAMPLES)
public class MecanumDriveOpMode extends PluginOpMode {
    @Override
    public RobotPlugin[] initPlugins() {
        MecanumDrive mecanumDrive = new MecanumDrive(this);
        return new RobotPlugin[]{mecanumDrive};
    }
}