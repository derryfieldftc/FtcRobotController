package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

public class DSLog {
    PrintWriter out;

    // constructor
    public DSLog (String path) {
        File file = new File(path);

        try {
            out = new PrintWriter(file);
        } catch (FileNotFoundException e) {
            RobotLog.e("Unable to open DSLog file: " + e.getMessage());
        }
    }

    public void close() {
        out.close();
    }

    public void log(String msg) {
        out.println(msg);
    }
}
