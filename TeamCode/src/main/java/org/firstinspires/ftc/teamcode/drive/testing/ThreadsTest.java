package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ThreadsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Thread thread = new Thread(new Runnable1());
        thread.start();
        Thread.sleep(1000);
    }

    class Runnable1 implements Runnable {

        @Override
        public void run() {

        }
    }
}
