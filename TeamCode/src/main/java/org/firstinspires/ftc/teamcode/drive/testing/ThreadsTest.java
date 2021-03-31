package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(group = "testing")
public class ThreadsTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (opModeIsActive()) {
            Thread thread = new Thread(new Runnable1());
            thread.start();
            Thread.sleep(1000);
        }
    }

    class Runnable1 implements Runnable {

        @Override
        public void run() {
            sleep(5000);
        }
    }
}
