
# Plugin Oriented Design

### Modular OpModes

For ease of implementation, the `PluginOpMode` abstract class exists. This class manages
multiple instances of `RobotPlugin`. In order to create an `OpMode` which uses plugins, 
use the template found in `ExampleOpMode.java`:

### Plugins

An instance of `RobotPlugin` exists to do *one* thing. These plugins can be composed into an `OpMode`
by extending the `PluginOpMode` class, as seen in `ExampleOpMode.java`.

Refer to `ExamplePlugin` for how to implement a plugin for robot behaviour.

---
# Standardizations

### Motor Naming
The names of stored `DcMotor` variables are the exact same as the configured names on every Control Hub.
Back Left = motorBL
Back Right = motorBR
Front Left = motorFL
Front Right = motorFR

### OpMode Naming
ALl classes in the `org.firstinspires.ftc.teamcode.opmodes` package must match `^*OpMode$`
