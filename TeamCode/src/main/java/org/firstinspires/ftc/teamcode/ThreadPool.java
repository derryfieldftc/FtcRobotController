package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class ThreadPool {
    ArrayList<Thread> threads;
    public ThreadPool() {
        this.threads = new ArrayList<>();
    }
    public void addThread(Thread thread) {
        this.threads.add(thread);
    }
    public void startAll() {
        for (Thread thread : threads) {
            thread.start();
        }
    }
    public void joinAll() throws InterruptedException {
        for (Thread thread : threads) {
            thread.join();
        }
    }
}
