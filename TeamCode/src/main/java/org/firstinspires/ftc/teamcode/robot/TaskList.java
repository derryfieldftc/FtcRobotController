package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.Tasks.OldTask;
import org.firstinspires.ftc.teamcode.robot.Tasks.TaskType;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Scanner;

public class TaskList {
    ArrayList<OldTask> tasks = null;

    public TaskList() {
        tasks = new ArrayList<>();
    }

    // import a sequence of tasks from a file.
    // return the sequence as an array list with first task at index = 0.
    public void importTasks(String path) {
        ArrayList <OldTask> tasks = new ArrayList<>();
        File file = new File(path);
        try(Scanner in = new Scanner(file, StandardCharsets.UTF_8.name())) {
            while(in.hasNextLine()) {
                String line = in.nextLine();
                line = line.replaceAll("[^a-zA-Z0-9.\\-,_\\s]", "");
                if (line.length() < 1) {
                    continue;
                }
                Scanner data = new Scanner(line);
                data.useDelimiter("[\\s,\\t\\n,]+");
                // read token, then extract type. if applicable, additional information.
                String token = data.next().trim();
                TaskType type = OldTask.type(token);

                OldTask task = new OldTask(type);

                // do we need to get additional data?
                if (type == TaskType.WAYPOINT) {
                    double x, y, theta;
                    x = RobotData.getDouble(data);
                    y = RobotData.getDouble(data);
                    theta = RobotData.getDouble(data);
                    Pose pose = new Pose(x, y, theta);
                    task.setPose(pose);
                } else if (type == TaskType.DELAY) {
                    long period = RobotData.getLong(data);
                    task.setPeriod(period);
                }

                // add to list of tasks.
                tasks.add(task);
            }
        } catch(FileNotFoundException e) {
            RobotLog.e("importTasks error: " + e.getMessage());
        }

        this.tasks = tasks;
    }

    public ArrayList<OldTask> get() {
        return tasks;
    }

    public void exportTasks(String path) throws IOException {
        File file = new File(path);
        file.createNewFile();
        PrintWriter writer = new PrintWriter(file);
        for (OldTask task : tasks) {
            if (task.getType() == TaskType.DELAY) {
                writer.println(String.format("DELAY: %d", task.getPeriod()));
            } else if (task.getType() == TaskType.WAYPOINT) {
                Pose pose = task.getPose();
                writer.println(String.format("WAYPOINT: %.6f %.6f %.6f", pose.x, pose.y, pose.theta));
            }
        }
        writer.flush();
        writer.close();
    }

    public int size() {
        return tasks.size();
    }

    public OldTask nextTask() {
        OldTask task = null;
        if (tasks.size() > 0) {
            task = tasks.get(0);
            tasks.remove(0);
        }
        return task;
    }

    public void add(OldTask task) {
        tasks.add(task);
    }

    public void clear() {
        tasks.clear();
    }

    public OldTask get(int index) {
        return tasks.get(index);
    }

    public void remove(int index) {
        tasks.remove(index);
    }

    public String toString() {
        String s = "";

        for(int i = 0; i < tasks.size(); i++) {
            s = s + tasks.get(i).toString();
            if (i < tasks.size() - 1) {
                s = s + ", ";
            }
        }

        return s;
    }
}
