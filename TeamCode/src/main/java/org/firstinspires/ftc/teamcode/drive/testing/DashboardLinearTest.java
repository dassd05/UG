package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;
import java.util.List;

@Disabled
@Config("DashboardTestL")
@TeleOp(name = "DashboardTestL", group = "testing")
public class DashboardLinearTest extends LinearOpMode {

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

    private BNO055IMU imu;
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

    public static boolean squareShow = false;
    public static int squareX = 0;
    public static int squareY = 0;
    public static int squareSize = 25;

    public static boolean sphereShow = false;
    public static int sphereX = 0;
    public static int sphereY = 0;
    public static int sphereSize = 25;
    public static int sphereLightPointX = -10;
    public static int sphereLightPointY = -10;
    public static double sphereLightIntensity = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "imu";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(imuParameters);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive() && !isStopRequested()) {

            if (squareShow) {
                for (int x = 0; x < squareSize; x++) {
                    for (int y = 0; y < squareSize; y++) {
                        int c = (x + y) * 255 / squareSize / 2;
                        canvas.setFill(color(c, c, c));
                        int toMid = squareSize / 2;
                        canvas.fillRect(x - toMid, y - toMid, 1, 1);
                    }
                }
            }
            if (sphereShow) {
                for (int x = 0; x < sphereSize; x ++) {
                    for (int y = 0; y < sphereSize; y ++) {
                        int c = Range.clip(255 - (int)(sphereLightIntensity * Math.sqrt(Math.pow(x - sphereLightPointX, 2) + Math.pow(y - sphereLightPointY, 2))), 0, 255);
                        canvas.setFill(color(c, c, c));
                        int toMid = sphereSize / 2;
                        canvas.fillRect(x - toMid - sphereX, y - toMid - sphereY, 1, 1);
                    }
                }
            }

            canvas.fillRect(0, 0, 10, 30);


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
            telemetry_addData("angle", imu.getAngularOrientation().firstAngle);
            telemetry_addData("position", imu.getPosition());
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
        String redS = Integer.toHexString(red);
        String greenS = Integer.toHexString(green);
        String blueS = Integer.toHexString(blue);
        List<String> strings = Arrays.asList(redS, greenS, blueS);
        StringBuilder hexColor = new StringBuilder("#");
        for (String str : strings) {
            if (str.length() > 2) str = str.substring(str.length() - 2);
            else if (str.length() == 1) str = "0" + str;
            hexColor.append(str);
        }
        return hexColor.toString();
    }

    class CanvasEx extends Canvas {
        @Override
        public Canvas strokeCircle(double x, double y, double radius) {
            return super.strokeCircle(-y, x, radius);
        }

        @Override
        public Canvas fillCircle(double x, double y, double radius) {
            return super.fillCircle(-y, x, radius);
        }

        @Override
        public Canvas strokePolygon(double[] xPoints, double[] yPoints) {
            double[] newPointsY = new double[yPoints.length];
            for (int i = 0; i < yPoints.length; i ++) {
                newPointsY[i] = -yPoints[i];
            }
            return super.strokePolygon(newPointsY, xPoints);
        }

        @Override
        public Canvas fillPolygon(double[] xPoints, double[] yPoints) {
            double[] newPointsY = new double[yPoints.length];
            for (int i = 0; i < yPoints.length; i ++) {
                newPointsY[i] = -yPoints[i];
            }
            return super.fillPolygon(newPointsY, xPoints);
        }

        @Override
        public Canvas strokePolyline(double[] xPoints, double[] yPoints) {
            double[] newPointsY = new double[yPoints.length];
            for (int i = 0; i < yPoints.length; i ++) {
                newPointsY[i] = -yPoints[i];
            }
            return super.strokePolyline(newPointsY, xPoints);
        }

        @Override
        public Canvas strokeLine(double x1, double y1, double x2, double y2) {
            return super.strokeLine(-y1, x1, -y2, x2);
        }

        @Override
        public Canvas fillRect(double x, double y, double width, double height) {
            return super.fillRect(-y, x, width, height);
        }

        @Override
        public Canvas strokeRect(double x, double y, double width, double height) {
            return super.strokeRect(-y, x, width, height);
        }
    }
}
