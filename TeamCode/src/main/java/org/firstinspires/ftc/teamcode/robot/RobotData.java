package org.firstinspires.ftc.teamcode.robot;

import java.util.Scanner;

/*
 * Use this class to store/retrieve configuration info and data for the robot.
 */
public class RobotData {
    public static String cleanToken(String token) {
        // trim white space.
        token = token.trim();
        // remove any non alphanumeric characters (except . and -).
        token = token.replaceAll("[^a-zA-Z0-9\\.\\-]", "");
        return token;
    }

    public static double getDouble(Scanner in) {
        String token = cleanToken(in.next());
        double value = Double.parseDouble(token);
        return value;
    }

    public static long getLong(Scanner in) {
        String token = cleanToken(in.next());
        long value = Long.parseLong(token);
        return value;
    }
}
