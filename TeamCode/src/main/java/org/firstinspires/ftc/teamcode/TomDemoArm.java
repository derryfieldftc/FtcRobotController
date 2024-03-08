package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.demo.FiveDOFArm;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Demo Arm", group = "Tom")
public class TomDemoArm extends OpMode {
    private FiveDOFArm arm;
    private boolean dpad_right_previous = false;
    private boolean dpad_left_previous = false;

    public void nextState() {
        FiveDOFArm.ArmState currentState = arm.getState();
        switch (currentState) {
            case NEUTRAL:
                arm.setState(FiveDOFArm.ArmState.PREPICK);
                break;
            case PREPICK:
                arm.setState(FiveDOFArm.ArmState.PICK);
                break;
            case PICK:
                arm.setState(FiveDOFArm.ArmState.PICKUP);
                break;
            case PICKUP:
                arm.setState(FiveDOFArm.ArmState.TURN);
                break;
            case TURN:
                arm.setState(FiveDOFArm.ArmState.EXTEND);
                break;
            case EXTEND:
                arm.setState(FiveDOFArm.ArmState.PREPLACE);
                break;
            case PREPLACE:
                arm.setState(FiveDOFArm.ArmState.PLACE);
                break;
            case PLACE:
                arm.setState(FiveDOFArm.ArmState.POSTPLACE);
                break;
            case POSTPLACE:
                arm.setState(FiveDOFArm.ArmState.NEUTRAL);
                break;
        }
    }

    public void previousState() {
        FiveDOFArm.ArmState currentState = arm.getState();
        switch (currentState) {
            case POSTPLACE:
                arm.setState(FiveDOFArm.ArmState.PLACE);
                break;
            case PLACE:
                arm.setState(FiveDOFArm.ArmState.PREPLACE);
                break;
            case PREPLACE:
                arm.setState(FiveDOFArm.ArmState.EXTEND);
                break;
            case EXTEND:
                arm.setState(FiveDOFArm.ArmState.TURN);
                break;
            case TURN:
                arm.setState(FiveDOFArm.ArmState.PICKUP);
                break;
            case PICKUP:
                arm.setState(FiveDOFArm.ArmState.PICK);
                break;
            case PICK:
                arm.setState(FiveDOFArm.ArmState.PREPICK);
                break;
            case PREPICK:
                arm.setState(FiveDOFArm.ArmState.NEUTRAL);
                break;
            case NEUTRAL:
                arm.setState(FiveDOFArm.ArmState.POSTPLACE);
                break;
        }
    }


    @Override
    public void init() {
        arm = new FiveDOFArm(hardwareMap);
        arm.setState(FiveDOFArm.ArmState.NEUTRAL);
    }

    @Override
    public void loop() {
        /*
         * Check if a dpad right was just pressed.
         */
        if (gamepad1.dpad_right && dpad_right_previous == false) {
            // button was just pressed.
            dpad_right_previous = true;
            nextState();
        } else if (gamepad1.dpad_right == false && dpad_right_previous == true) {
            // button was just released.
            dpad_right_previous = false;
        }

        /*
         * Check if a dpad left was just pressed.
         */
        if (gamepad1.dpad_left && dpad_left_previous == false) {
            // button was just pressed.
            dpad_left_previous = true;
            previousState();
        } else if (gamepad1.dpad_left == false && dpad_left_previous == true) {
            // button was just released.
            dpad_left_previous = false;
        }

        // update arm
        arm.updateArm();

        // send telemetry info.
        telemetry.addData("state", arm.getState());
        telemetry.update();
    }
}
