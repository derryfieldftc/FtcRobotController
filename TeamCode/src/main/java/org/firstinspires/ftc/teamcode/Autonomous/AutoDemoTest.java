package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.CenterStageRobot;
@Autonomous(name = "AutoDemoTest")
public class AutoDemoTest extends LinearOpMode {
    CenterStageRobot moose;
    @Override
    public void runOpMode() throws InterruptedException {
        moose = new CenterStageRobot(this);
        moose.initHardware();

        waitForStart();
        if(opModeIsActive()) {
            moose.drive(-20);
            moose.drive(20);
            moose.strafe(20);
            moose.strafe(-20);
            moose.rotate(90);
            moose.rotate(-90);
        }
    }
}