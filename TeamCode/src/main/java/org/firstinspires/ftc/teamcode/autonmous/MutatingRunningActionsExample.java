package org.firstinspires.ftc.teamcode.autonmous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonmous.actions.Action;
import org.firstinspires.ftc.teamcode.autonmous.actions.SequentialAction;
import org.firstinspires.ftc.teamcode.autonmous.actions.SleepAction;


/**
 * This class shows an example of how to make an action change itself midway through operation. This can be made more complex with the getAction() method from the SequetialAction class, and the WrappedAction class
 */
@Autonomous
public class MutatingRunningActionsExample extends OpMode {
    // This should look familiar
    boolean running = true;

    // This is where the interesting things happen
    // first we define but do not initialize our mainAction
    SequentialAction mainAction;

    // Next we define and initialize the actions we wish to add partway through the auto
    Action secondAction = new Action() {
        @Override
        public boolean run() {
            telemetry.addLine("Ran Second Action");
            return false;
        }
    };
    Action thirdAction = new Action() {
        @Override
        public boolean run() {
            telemetry.addLine("Ran Third Action");
            return false;
        }
    };

    // This is the action that actually adds it, right now it will randomly add either the second or third action obviously much more could be done
    Action actionOne = new Action() {
        @Override
        public boolean run() {
            telemetry.addLine("Ran First Action");
            if (Math.random() > .5) {
                mainAction.appendActions(secondAction);
            } else {
                mainAction.appendActions(thirdAction);
            }
            return false;
        }
    };

    // All the rest is normal
    @Override
    public void init() {
        mainAction = new SequentialAction(new SleepAction(1000), actionOne, new SleepAction(1000));
    }

    @Override
    public void loop() {
        if (running) {
            running = mainAction.run();
        }
        telemetry.update();
    }
}
