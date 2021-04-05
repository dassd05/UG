package org.firstinspires.ftc.teamcode.drive.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.drive.auton.AutonRingDetecting.RingDetecting.RingDetection.avg1;
import static org.firstinspires.ftc.teamcode.drive.auton.AutonRingDetecting.RingDetecting.pipeline;

@Autonomous(name = "AutonRingDetecting", group = "sensor")
public class AutonRingDetecting extends LinearOpMode {

    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;

    public enum HowManyRings {
        FourRings,
        OneRing,
        ZeroRings,
        Default
    }

    public volatile HowManyRings ThisManyRings;

    private DcMotorEx frontShoot, backShoot;

    @Override
    public void runOpMode() {

        frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");
        backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        while (!opModeIsActive()) {
            if (pipeline.position == null) {
                telemetry.addLine("still working on it");
                ThisManyRings = HowManyRings.Default;
            } else if (pipeline.position == RingDetecting.RingPosition.FOUR){
                telemetry.addLine("Four Rings. Waiting for start");
                ThisManyRings = HowManyRings.FourRings;
            }
            else if (pipeline.position == RingDetecting.RingPosition.ONE){
                telemetry.addLine("One Ring. Waiting for start");
                ThisManyRings = HowManyRings.OneRing;
            }
            else if (pipeline.position == RingDetecting.RingPosition.NONE){
                telemetry.addLine("Zero Rings. Waiting for start");
                ThisManyRings = HowManyRings.ZeroRings;
            }
        }
        telemetry.addData("Number of Rings", ThisManyRings);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()){
            while (opModeIsActive()) {
                switch (ThisManyRings) {
                    case FourRings:
                        telemetry.addLine("4 rings detected; wobble position C");
                        frontShoot.setPower(.2);
                        backShoot.setPower(.2); // to test if it's working
                        //autonFourRings();
                        break;
                    case OneRing:
                        telemetry.addLine("1 ring detected; wobble position B");
                        frontShoot.setPower(-.2);
                        backShoot.setPower(-.2); // to test if it's working
                        //autonOneRing();
                        break;
                    case ZeroRings:
                        telemetry.addLine("0 rings detected; wobble position A");
                        frontShoot.setPower(.5);
                        backShoot.setPower(.5); // to test if it's working
                        //autonZeroRings();
                        break;
                    case Default:
                        telemetry.addData("ERROR", "NUMBER OF RINGS NOT RECOGNIZED");
                        frontShoot.setPower(0);
                        backShoot.setPower(0); // to test if it's working
                        break;
                }
            }
        }
        telemetry.addData("run", "finished");
        telemetry.update();
    }

    public abstract static class RingDetecting extends LinearOpMode {
        protected WebcamName webcamName;
        protected OpenCvWebcam webcam;


        public void webcamInitialize() {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
                }
            });
        }

        public int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        protected static RingDetection pipeline = new RingDetection();

        // An enum to define the number of rings
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        public static class RingDetection extends OpenCvPipeline {

            // Some color constants
            private final Scalar BLUE = new Scalar(0, 0, 255);
            private final Scalar GREEN = new Scalar(0, 255, 0);

            // The core values which define the location and size of the sample regions
            private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(170, 315);

            private final int REGION_WIDTH = 180;
            private final int REGION_HEIGHT = 150;

            private int FOUR_RING_THRESHOLD = 145;
            private int ONE_RING_THRESHOLD = 132;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            // Working variables
            Mat region1_Cb;
            Mat YCrCb = new Mat();
            Mat Cb = new Mat();
            static int avg1;

            // Volatile since accessed by OpMode thread w/o synchronization
            public volatile RingPosition position;
            private boolean yes;

            // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
            void inputToCb(Mat input) {
                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(YCrCb, Cb, 1);
            }

            @Override
            public void init(Mat firstFrame) {
                inputToCb(firstFrame);

                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            }

            @Override
            public Mat processFrame(Mat input) {
                    inputToCb(input);

                    avg1 = (int) Core.mean(region1_Cb).val[0];

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            BLUE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines


                    if (avg1 > FOUR_RING_THRESHOLD) {
                        position = RingPosition.FOUR;
                    } else if (avg1 > ONE_RING_THRESHOLD) {
                        position = RingPosition.ONE;
                    } else {
                        position = RingPosition.NONE;
                    }

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            GREEN, // The color the rectangle is drawn in
                            -1); // Negative thickness means solid fill

                    return input;
            }
        }
    }
}