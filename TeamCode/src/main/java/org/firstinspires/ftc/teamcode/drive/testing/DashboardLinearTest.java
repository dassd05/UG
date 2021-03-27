package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(group = "testing")
public class DashboardLinearTest extends LinearOpMode {

    private VoltageSensor voltageSensor;

    private FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    Canvas canvas = packet.fieldOverlay();

    long startTime = System.currentTimeMillis();
    long timeElapsed = startTime;
    int frameCount = 0;
    int fps = 0;
    long lastFrame = System.nanoTime();
    long timeBetweenFrames;

    @Override
    public void runOpMode() throws InterruptedException {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            frameCount++;
            if (System.currentTimeMillis() - timeElapsed > 1000) {
                fps = frameCount;
                frameCount = 0;
                timeElapsed = System.currentTimeMillis();
            }
            timeBetweenFrames = System.nanoTime() - lastFrame;
            lastFrame = System.nanoTime();

            telemetry_addData("battery", voltageSensor.getVoltage());
            telemetry_addData("fps", fps);
            telemetry_addData("timeBetweenFrames", (int)(timeBetweenFrames/1000));
            telemetry_addData("frameCount", frameCount);
//        telemetry_addData("angle", imu.getAngularOrientation().firstAngle);
//        telemetry_addData("position", imu.getPosition());
            telemetry_update();

//        telemetry = new MultipleTelemetry(telemetry);
        }
    }

    public void telemetry_addData(String caption, Object value) {
        telemetry.addData(caption, value);
        packet.put(caption, value);
    }
    public void telemetry_update() {
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        canvas = packet.fieldOverlay();
    }

    public String color(int red, int green, int blue) {
        return "#" + Integer.toHexString(red) + Integer.toHexString(green) + Integer.toHexString(blue);
    }
}
