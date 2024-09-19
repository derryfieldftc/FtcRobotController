package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeGroups;
import org.firstinspires.ftc.teamcode.RobotPlugin;
import org.firstinspires.ftc.teamcode.plugins.Claw;

@TeleOp(name = "ClawTestOpMode", group = OpModeGroups.TESTS)
public class ClawTestOpMode extends PluginOpMode {
        @Override
        public RobotPlugin[] initPlugins() {
            Claw claw = new Claw(this);
            Claw.config config = claw.new config();
            config.rightServo("right", Servo.Direction.FORWARD, 1, 0);
            config.leftServo("left", Servo.Direction.REVERSE, 1, 0);
            return new RobotPlugin[]{ claw };
        }
}
