package org.firstinspires.ftc.teamcode;

public abstract class RobotPlugin {
    // Code is run ONCE when the driver hits INIT
    public void init() {}

    // Code is run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    public void init_loop() {}

    // Code is run ONCE when the driver hits PLAY
    public void start() {}

    // Code is run REPEATEDLY after the driver hits PLAY but before they hit STOP
    public void loop() {}

    // Code to run ONCE after the driver hits STOP
    public void stop() {}
}
