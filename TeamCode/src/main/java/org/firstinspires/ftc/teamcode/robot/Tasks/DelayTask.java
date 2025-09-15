package org.firstinspires.ftc.teamcode.robot.Tasks;

import java.time.Period;
import java.util.Scanner;

public class DelayTask implements Task {
	int periodms;

	public DelayTask(int periodms) {
		this.periodms = periodms;
	};

	@Override
	public String asString() {
		return "DELAY " + periodms;
	}

	@Override
	public boolean isParseable(String input) {
		return input.contains("DELAY");
	}

	@Override
	public Task parse(String input) {
		Scanner parser = new Scanner(input.substring(5));
		return new DelayTask(parser.nextInt());
	}

	/**
	 * This IS blocking so be careful
	 */
	//TODO! make a lil cleaner
	@Override
	public void execute() {
		try {
			Thread.sleep(periodms);
		} catch (InterruptedException e) {
//			throw new RuntimeException(e);
		}
	}

	@Override
	public TaskType getType() {
		return TaskType.DELAY;
	}
}
