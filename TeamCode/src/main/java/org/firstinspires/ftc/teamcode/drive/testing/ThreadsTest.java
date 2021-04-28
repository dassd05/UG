package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(group = "testing")
public class ThreadsTest extends LinearOpMode {

    long time;
    FtcDashboard dashboard;
    Telemetry dashTelemetry;

    Servo servo0, servo1, servo2, servo3;

    public static boolean withThreads = true;

    @Override
    public void runOpMode() throws InterruptedException {
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        dashboard = FtcDashboard.getInstance();

        dashTelemetry = dashboard.getTelemetry();

        dashTelemetry.setAutoClear(false);
        telemetry.setAutoClear(false);

        waitForStart();

        double pos = servo0.getPosition() < 0.1 ? 1 : 0;
        Thread t1 = new Thread(() -> {
            multiTelemetry("start_time0", System.currentTimeMillis() - time);
            servo0.setPosition(pos);
            multiTelemetry("end_time0", System.currentTimeMillis() - time);
            telemetry.update();
        });
        Thread t2 = new Thread(() -> {
            multiTelemetry("start_time1", System.currentTimeMillis() - time);
            servo1.setPosition(pos);
            multiTelemetry("end_time1", System.currentTimeMillis() - time);
            telemetry.update();
        });
        Thread t3 = new Thread(() -> {
            multiTelemetry("start_time2", System.currentTimeMillis() - time);
            servo2.setPosition(pos);
            multiTelemetry("end_time2", System.currentTimeMillis() - time);
            telemetry.update();
        });

        if (opModeIsActive()) {
            time = System.currentTimeMillis();

            multiTelemetry("start_time", System.currentTimeMillis() - time);
            if (withThreads) {
                t1.start();
                t2.start();
                t3.start();
                while (t1.isAlive() || t2.isAlive() || t2.isAlive()) {
                    //sleep(1);
                }
            } else {
                multiTelemetry("start_time0", System.currentTimeMillis() - time);
                servo0.setPosition(pos);
                multiTelemetry("end_time0", System.currentTimeMillis() - time);
                multiTelemetry("start_time1", System.currentTimeMillis() - time);
                servo1.setPosition(pos);
                multiTelemetry("end_time1", System.currentTimeMillis() - time);
                multiTelemetry("start_time2", System.currentTimeMillis() - time);
                servo2.setPosition(pos);
                multiTelemetry("end_time2", System.currentTimeMillis() - time);
            }
            multiTelemetry("end_time", System.currentTimeMillis() - time);
            telemetry.update();
            dashTelemetry.update();
            sleep(2500);
        }
    }


    /*
    This is able to asynchronously queue shots. So if you queueShot x times rapidly,
    it will shoot x times successively, even if it's still shooting
     */
    private int shotsQueued = 0;
    protected Thread shoot = new Thread(() -> {
        try {
            servo0.setPosition(1);
            Thread.sleep(1000);
            servo0.setPosition(0);
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            servo0.setPosition(0);
            Thread.currentThread().interrupt();
        }
    });
    public void queueShot() {
        shotsQueued ++;
    }
    public void update() {
        if (!shoot.isAlive() && shotsQueued > 0) {
            shotsQueued --;
            shoot.start();
        }
    }

    private void multiTelemetry(String caption, Object value) {
        telemetry.addData(caption, value);
        dashTelemetry.addData(caption, value);
    }

}
