// description: Same as prior but uses Opmode instead of LinearOpMode i guess.
package org.firstinspires.ftc.teamcode.sideQuests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Edge Detect Test",group="Side Quests")
public class NonLinearEdgeTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    boolean xState = false;

    @Override
    public void init() {
        //standard waiting for START.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void loop() {
        runtime.reset();

            if (gamepad1.x && !xState) {
                    xState = true;
            } else if (!gamepad1.x && xState) {
                    xState = false;
            }

            telemetry.addData("X is Pressed:", xState);
            telemetry.update();

    }
}
