package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Helper.Camera;
import org.firstinspires.ftc.teamcode.Helper.MecanumDrive;

@Autonomous(name="Limit Switch Test", group="Tests")
public class LimitSwitchTest extends LinearOpMode {

    private TouchSensor limit;
    @Override
    public void runOpMode() {

        TouchSensor limit = hardwareMap.get(TouchSensor.class, "limit");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Limit Switch State", limit.isPressed());
            telemetry.update();
        }
    }
}

