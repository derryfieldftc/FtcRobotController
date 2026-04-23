// description: An op-mode that detects when x is pressed. while x pressed, "state"
// variable is on. While not, it's off. Use telemetry to display its functionality.

package org.firstinspires.ftc.teamcode.sideQuests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Edge Detect Test",group="Side Quests")
public class EdgeTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    boolean xState = false;

    @Override
    public void runOpMode() {
        //standard waiting for START.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            if (gamepad1.x && !xState) {
                    xState = true;
            } else if (!gamepad1.x && xState) {
                    xState = false;
            }

            telemetry.addData("X is Pressed:", xState);
            telemetry.update();
        }
    }
}
