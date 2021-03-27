package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;


//@Config
@TeleOp(group = "testing")
public class DashboardTest extends OpMode {

    /*
    STUFF:
    telemetry packets are alphabetically sorted (the data is stored in TreeMaps)
        want to change it to HashMaps
    the integration between dashboard and normal isn't seamless enough
        you often have to do things double
        good idea to make a base class to work off of
    why make the changeable variables static
        can't you use an annotation
        is there a better way to do it
        or is it good enough
    no float support
    how to customize
    how to adapt
    how to lower latency
    what is the refresh rate
    how to modify graphs
    how to make better

    LINKS:
    https://acmerobotics.github.io/ftc-dashboard/
    https://github.com/adamrubinfeld/Dashboard
    https://github.com/FRCDashboard/FRCDashboard
     */

//    private BNO055IMU imu;
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

    public static int objectX = 50;
    public static int objectY = 50;

//    @Config
//    static class Dependencies {
//        public static double object1x = 0;
//        public static double object1y = 0;
//        public static double object1s = 10;
//        public static double object2x = 0;
//        public static double object2y = 0;
//        public static double object2s = 20;
//        public static boolean show = true;
//        public static shapeType object1st = shapeType.CIRCLE;
//        public static shapeType object2st = shapeType.RECT;
//
//        public enum shapeType {
//            CIRCLE,
//            RECT
//        }
//
//    }

    @Override
    public void init() {
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
//        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        imuParameters.loggingEnabled = true;
//        imuParameters.loggingTag = "imu";
//        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(imuParameters);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {
        frameCount++;
        if (System.currentTimeMillis() - timeElapsed > 1000) {
            fps = frameCount;
            frameCount = 0;
            timeElapsed = System.currentTimeMillis();
        }
        timeBetweenFrames = System.nanoTime() - lastFrame;
        lastFrame = System.nanoTime();

//        canvas.setStrokeWidth(1);
//        canvas.setStroke(color(0, 0, 0));
//        canvas.setFill(color(50, 220, 200));
//        canvas.fillCircle(objectX, objectY, 10);
////        canvas.strokeCircle(20, 40, 5);
//        if (Dependencies.show) {
//            canvas.setStrokeWidth(3);
//            canvas.setStroke(color(200, 50, 170));
//            canvas.setFill(color(90, 100, 120));
//            double s1 = Dependencies.object1s;
//            if (Dependencies.object1st == Dependencies.shapeType.CIRCLE) canvas.fillCircle(Dependencies.object1x, Dependencies.object1y, s1/2);
//            else if (Dependencies.object1st == Dependencies.shapeType.RECT) canvas.fillRect(Dependencies.object1x - s1/2, Dependencies.object1y - s1/2, s1, s1);
//            double s2 = Dependencies.object2s;
//            if (Dependencies.object2st == Dependencies.shapeType.CIRCLE) canvas.strokeCircle(Dependencies.object2x, Dependencies.object2y, s2/2);
//            else if (Dependencies.object2st == Dependencies.shapeType.RECT) canvas.strokeRect(Dependencies.object2x - s2/2, Dependencies.object2y - s2/2, s2, s2);
//        }

        telemetry_addData("battery", voltageSensor.getVoltage());
        telemetry_addData("fps", fps);
        telemetry_addData("timeBetweenFrames", (int)(timeBetweenFrames/1000));
        telemetry_addData("frameCount", frameCount);
//        telemetry_addData("angle", imu.getAngularOrientation().firstAngle);
//        telemetry_addData("position", imu.getPosition());
        telemetry_update();

//        telemetry = new MultipleTelemetry(telemetry);
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
