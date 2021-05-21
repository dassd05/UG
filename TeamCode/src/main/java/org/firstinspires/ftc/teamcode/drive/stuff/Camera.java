package org.firstinspires.ftc.teamcode.drive.stuff;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * A camera class to handle vuforia and ring handling easier. Create an object of this to use it,
 * or inherit it and add a inner class for another implementation of camera.
 *
 * @author Ethan
 */
@SuppressWarnings({"deprecation", "unused"})
public class Camera {

    /*
    TODO:
     ring sensing pipeline
     calculating position (maybe weighted, maybe just take closest)
     */

    //----------------------------------------------------------------------------------------------
    // CONSTANTS/DEFAULTS
    //----------------------------------------------------------------------------------------------

    public static final String DEFAULT_CAM_NAME = "Webcam 1";

    //----------------------------------------------------------------------------------------------
    // FIELDS
    //----------------------------------------------------------------------------------------------

    public WebcamName webcamName;
    public OpenCvWebcam cvWebcam;
    public int cameraMonitorViewId;

    /**
     * The instance of Vuforia currently being used in the Camera.
     *
     * <p>Will be null if TrackingType.VUFORIA isn't given as a parameter in the constructor. If
     * this is the case, DON'T try to use vuforia or make a new instance.</p>
     */
    public Vuforia vuforia;

    public CamType camType;

    /**
     * The instance of RingDetection currently being used in the Camera.
     *
     * <p>Will be null if TrackingType.RING_DETECTION isn't given as a parameter in the constructor.
     * If this is the case, DON'T try to use RingDetection of make a new instance.</p>
     */
    public RingDetection pipeline;

    Set<TrackingType> trackers = new HashSet<>();


    public float forwardDisplacement = 4.0f * 25.4f;   // eg: Camera is 4 Inches in front of robot-center
    public float horizontalDisplacement = 0;     // eg: Camera is ON the robot's center line
    public float verticalDisplacement = 8.0f * 25.4f;   // eg: Camera is 8 Inches above ground

    protected float xAngleDisplacement = 0;
    protected float yAngleDisplacement = -90;
    protected float zAngleDisplacement = 0;


    //----------------------------------------------------------------------------------------------
    // ENUMS
    //----------------------------------------------------------------------------------------------

    public static abstract class CamType {

        public enum Side {
            FRONT, BACK
        }

        public enum Orientation {
            PORTRAIT, LANDSCAPE
        }

        public static class Webcam extends CamType {
            public String name;
            public String vuforiaKey;
            public Side side = Side.BACK;
            public Orientation orientation = Orientation.LANDSCAPE;

            public Webcam(@Nullable String name, @Nullable String vuforiaKey) {
                this.name = name;
                this.vuforiaKey = vuforiaKey;
            }
            public Webcam() {}
        }

        @Deprecated
        public static class Phone extends CamType {
            public Side side;
            public Orientation orientation;

            public Phone(Side side, Orientation orientation) {
                this.side = side;
                this.orientation = orientation;
            }
        }
    }

    public enum TrackingType {
        RING_DETECTION, VUFORIA
    }

    // An enum to define the number of rings
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }


    //----------------------------------------------------------------------------------------------
    // CONSTRUCTION
    //----------------------------------------------------------------------------------------------

    @TeleOp(group = "testing")
    public static class CameraTestInternal extends LinearOpMode {

        Camera camera;

        double xPos;
        double yPos;
        double zPos;
        double roll;
        double pitch;
        double heading;

        @Override
        public void runOpMode() throws InterruptedException {
            camera = new Camera(hardwareMap, new Camera.CamType.Webcam(null, null),  Camera.TrackingType.RING_DETECTION);

//        camera.vuforia = camera.new Vuforia();
//        Camera.Vuforia vuf = camera.new Vuforia();
//        vuf.activate();

        waitForStart();

//        camera.vuforia.activate();

            while(!isStopRequested()) {
//            camera.vuforia.update();
//            xPos = camera.vuforia.xPos;
//            yPos = camera.vuforia.yPos;
//            zPos = camera.vuforia.zPos;
//            roll = camera.vuforia.roll;
//            pitch = camera.vuforia.pitch;
//            heading = camera.vuforia.heading;
//
//            telemetry.addData("position", "{x, y, z} = %.0f, %.0f, %.0f", xPos, yPos, zPos);
//            telemetry.addData("angles", "{roll, pitch, heading} = %.0f, %.0f, %.0f",
//                    roll, pitch, heading);
                telemetry.addData("num rings", camera.pipeline.position);
                telemetry.addData("avg1", camera.pipeline.avg1);
                telemetry.addData("avg2", camera.pipeline.avgs2);
                telemetry.update();
            }

//            camera.vuforia.deactivate();
        }

    }


    // make a phone and webcam that inherits Camera, polymorphism
    Camera camera0 = new Camera(new HardwareMap(null), new CamType.Webcam("webcam 1", "key"), Arrays.asList(TrackingType.RING_DETECTION, TrackingType.VUFORIA));
    Camera camera1 = new Camera(new HardwareMap(null), new CamType.Webcam("webcam 1", "key"), new TrackingType[] {TrackingType.RING_DETECTION, TrackingType.VUFORIA});
    Camera camera2 = new Camera(new HardwareMap(null), new CamType.Webcam("webcam 1", "key"), TrackingType.RING_DETECTION, TrackingType.VUFORIA);

//    public Camera(HardwareMap hardwareMap, CamType.Webcam webcam, @Nullable TrackingType[] trackers) {}
    public Camera(HardwareMap hardwareMap, CamType.Webcam webcam, @Nullable TrackingType... trackers) {
        new Camera(hardwareMap, webcam, Arrays.asList(trackers));
    }

    /**
     *
     *
     * @param hardwareMap The hardwareMap in your OpMode so we can actually initialize a camera
     * @param webcam Webcam
     * @param trackers The things you want to track. Cannot be null here because I haven't
     *                 handled it yet, so if you don't want to track anything see
     *                 {@link #Camera(HardwareMap, CamType.Webcam, TrackingType...)}
     */
    public Camera(HardwareMap hardwareMap, CamType.Webcam webcam, @NonNull List<TrackingType> trackers) {
        // vuforia key = webcam.vuforiaKey;
        camType = webcam;
        this.trackers.addAll(trackers);

        this.webcamName = hardwareMap.get(WebcamName.class, webcam.name == null ? DEFAULT_CAM_NAME : webcam.name);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        setAngleDisplacement(0, 0, 0);
        setDisplacement(0, 0, 0);

        if (trackers.contains(TrackingType.VUFORIA)) {
            vuforia = new Vuforia();
        }

        if (trackers.contains(TrackingType.RING_DETECTION)) {
            cvWebcam = OpenCvCameraFactory.getInstance().createWebcam(this.webcamName, cameraMonitorViewId);
            cvWebcam.setPipeline(pipeline);
            cvWebcam.openCameraDeviceAsync(() -> cvWebcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT));

            //vuforiaCode.config();
        }
    }
    @Deprecated
    public Camera(HardwareMap hardwareMap, CamType.Phone phone, @Nullable List<TrackingType> trackers) {
    }
    public Camera(HardwareMap hardwareMap, String webcamName, @Nullable List<TrackingType> trackers) {
        if (camType instanceof CamType.Phone) {
            throw new IllegalArgumentException("CamType.Phone is deprecated, please don't use it");
        }


    }


    public void setDisplacement(double forward, double horizontal, double vertical) {
        forwardDisplacement = (float) forward;
        horizontalDisplacement = (float) horizontal;
        verticalDisplacement = (float) vertical;
    }
    public float[] getAngleDisplacements() {
        return new float[] {xAngleDisplacement - ((((CamType.Webcam) camType).orientation == CamType.Orientation.PORTRAIT) ? 90 : 0),
                yAngleDisplacement - ((((CamType.Webcam) camType).side == CamType.Side.FRONT) ? 90 : -90), zAngleDisplacement};
    }
    public void setAngleDisplacement(double xAngle, double yAngle, double zAngle) {
        xAngleDisplacement = (float) xAngle + ((((CamType.Webcam) camType).orientation == CamType.Orientation.PORTRAIT) ? 90 : 0);
        yAngleDisplacement = (float) yAngle + ((((CamType.Webcam) camType).side == CamType.Side.FRONT) ? 90 : -90);
        zAngleDisplacement = (float) zAngle;
    }


    public static double distanceFrom(OpenGLMatrix m1, OpenGLMatrix m2) {
        VectorF p1 = m1.getTranslation();
        VectorF p2 = m2.getTranslation();
        return Math.sqrt(sq(p1.get(0) - p2.get(0)) + sq(p1.get(1) - p2.get(1)) + sq(p1.get(2) - p2.get(2)));
    }
    public static double sq(double num) {
        return num * num;
    }


    //----------------------------------------------------------------------------------------------
    // CAMERA CLASSES
    //----------------------------------------------------------------------------------------------
    @SuppressWarnings("FieldCanBeLocal")
    public static class RingDetection extends OpenCvPipeline {

        // Some color constants
        protected final Scalar BLUE = new Scalar(0, 0, 255, 100);
        protected final Scalar GREEN = new Scalar(0, 255, 0, 100);

        // The core values which define the location and size of the sample regions
        private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(650, 400);

        private final int REGION_WIDTH = 150;
        private final int REGION_HEIGHT = 125;

        private final int FOUR_RING_THRESHOLD = 145;
        private final int ONE_RING_THRESHOLD = 130;

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
        public int avg1;

        Mat region_rgb;
        public double[] avgs2;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position;

        // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region_rgb = firstFrame.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];
            avgs2 = Core.mean(region_rgb).val;

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

        @Override
        public void onViewportTapped() {
            
        }
    }

    /**
     * The Vuforia class for the implementation of vuforia with the camera.
     */
    public class Vuforia {

        // constants
        public static final float mmPerInch        = 25.4f;
        public static final float mmTargetHeight   = (6) * mmPerInch;

        public static final float halfField = 72 * mmPerInch;
        public static final float quadField  = 36 * mmPerInch;

        public static final String DEFUALT_KEY = "AYeVcF3/////AAABmeH5+BbXYEN4qpSLicoaxc+IVmD28hHtX0bxb" +
                "efQtfKtYetF2JMES/y5/OB6AmulvNMR21cz8M5Sm0TaSIJ9VyiYtyQFy0DS0AwFaN5BGLmZoAIL9FVpY" +
                "udirubarSUaywQcQR9eyQNIOwlmsIRBjZU5WC1vBxUcZ8em+bF4kZMxx9W41i0eMz86/+iMuBYi/+mwY" +
                "VE0C4EM3NBUIw6XLwEL+hKI24ZWDd7i6uzPuw/Scgl8wbVEKxksgUTAdYQaioAVDZsqIMU33lmoOo0U2" +
                "GToSTu3LHnhbJv6RFa2sUH5ndBmgqbcdDavCyiXTGaWHqDc404amMpkUYzX3ZVO0UQaYbfJbfnSDrx8I" +
                "4uiL/FA";

        protected String vuforiaKey;

        protected VuforiaLocalizer vuforiaLocalizer;
        public VuforiaTrackables targets;

        /**
         * NOTE: if target isn't visible after updating, then the positional values from vuforia
         * are likely stale, and shouldn't be used.
         */

        public List<VuforiaTrackable> allTrackables;

        /**
         * Map containing the target name as the key and the raw position values given by vuforia
         * as the value.
         */
        public Map<String, OpenGLMatrix> visibleTargets;

        /**
         * The coordinates the robot is at, in inches, where each coordinate is of its respective
         * axis.
         *
         * <p>Note that these are based on the FTC's coordinates where x is positive towards the
         * back, y is positive towards the left, and z is positive upwards.</p>
         *
         * <p>Additionally, note that the origin is at the center of the (full) field, so at one
         * of the corners, your coordinates would be (+/-72, +/-72, 0)</p>
         */
        public double xPos, yPos, zPos;
        /**
         * The angles that the robot is at, in degrees. Roll, pitch, and heading are the rotations
         * around the x, y, and z axes, respectively.
         */
        public double roll, pitch, heading;


        public Vuforia() {
            VuforiaLocalizer.Parameters parameters;
            if (camType instanceof CamType.Webcam) {
                CamType.Webcam webcam = (CamType.Webcam) camType;
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = webcam.vuforiaKey == null ? DEFUALT_KEY : webcam.vuforiaKey;
                parameters.cameraName = webcamName;
                parameters.useExtendedTracking = false; // for some reason wasn't allowed in example
            } else if (camType instanceof CamType.Phone) {
                throw new IllegalArgumentException("CamType.Phone is deprecated, please don't use it");
            } else if (camType == null) {
                parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                parameters.vuforiaLicenseKey = DEFUALT_KEY;
                parameters.cameraName = webcamName;
                parameters.useExtendedTracking = false; // for some reason wasn't allowed in example
            } else {
                throw new IllegalArgumentException("unrecognized given camType");
            }

            vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
            createVuforiaTargets();

            OpenGLMatrix cameraFromRobot = OpenGLMatrix
                    .translation(forwardDisplacement, horizontalDisplacement, verticalDisplacement)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, xAngleDisplacement, yAngleDisplacement, zAngleDisplacement));

            /*  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                // for phone
//                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(cameraFromRobot, parameters.cameraDirection);
                // for webcam
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraFromRobot);
                // dunno the difference between the two
            }
        }

        public @Nullable OpenGLMatrix update() {
            visibleTargets = new HashMap<>();
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();
                    if (robotLocation == null) { continue; }

                    if (distanceFrom(robotLocation, trackable.getLocation()) < 50) {
                        visibleTargets.put(trackable.getName(), robotLocation);

                        VectorF v = robotLocation.getTranslation();
                        xPos = v.get(0) / mmPerInch;
                        yPos = v.get(1) / mmPerInch;
                        zPos = v.get(2) / mmPerInch;
                        Orientation o = Orientation.getOrientation(robotLocation, EXTRINSIC, XYZ, DEGREES);
                        roll = o.firstAngle;
                        pitch = o.secondAngle;
                        heading = o.thirdAngle;

                        return robotLocation;
                    }
                }
            }
            return null;

            /*
            TODO:
             check to see if camera is facing in right direction to target
             put greater weight if it is a straighter angle
             also greater weight if it is closer
             gotta get good way to find camera position
             switch to dealing everything in matrices?
             find way to find distance between matrices and direction of one is in other
             or matrices and points
             */

//            if (targetVisible) {
//                VectorF translation = lastLocation.getTranslation();
//                xPos = translation.get(0) / mmPerInch;
//                yPos = translation.get(1) / mmPerInch;
//                zPos = translation.get(2) / mmPerInch;
//
//                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                roll = rotation.firstAngle;
//                pitch = rotation.secondAngle;
//                heading = rotation.thirdAngle;
//            } else {
//                visibleTargets = null;
//            }
        }

        protected void createVuforiaTargets() {
            targets = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
            VuforiaTrackable blueTowerGoalTarget = targets.get(0);
            blueTowerGoalTarget.setName("Blue Tower Goal Target");
            VuforiaTrackable redTowerGoalTarget = targets.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");
            VuforiaTrackable redAllianceTarget = targets.get(2);
            redAllianceTarget.setName("Red Alliance Target");
            VuforiaTrackable blueAllianceTarget = targets.get(3);
            blueAllianceTarget.setName("Blue Alliance Target");
            VuforiaTrackable frontWallTarget = targets.get(4);
            frontWallTarget.setName("Front Wall Target");

            redAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
            blueAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            frontWallTarget.setLocation(OpenGLMatrix
                    .translation(-halfField, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
            blueTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            allTrackables = new ArrayList<>(targets);
        }

        public void activate() {
            targets.activate();
        }
        public void deactivate() {
            targets.deactivate();
        }

    }

}
