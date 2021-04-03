package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.*;

import static java.util.Arrays.copyOf;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class Robot extends LinearOpMode {
    protected DcMotor backLeft;
    protected DcMotor backRight;
    protected DcMotor frontRight;
    protected DcMotor frontLeft;
    protected DcMotor[] motors = {backLeft, backRight, frontRight, frontLeft}; // In port order
    //    protected DcMotor undefined;
    protected DcMotor frontShoot;
    protected DcMotor backShoot;
    protected DcMotor wobbleArm;
    protected Servo shootServo;
    protected Servo wobbleGoalServo;
    protected BNO055IMU imu;
    protected WebcamName webcamName;
    protected OpenCvWebcam webcam;

    private Orientation imuAngles;
    private Acceleration imuGravity;

    private double coordinateX = 0; // inches
    private double coordinateY = 0; // inches
    private double angle = 0; // degrees
    private double wobbleArmPosition = 90; // degrees

    private long startTime;
    private long timeElapsed = 0;
    private int frameCount = 0;
    private int fps = 0;

    private double turn = 0;

    double posX = 0; // inches
    double posY = 0;

    double lastAngle = 0;

    boolean RunToPoint = false;

    public double getAngle() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).firstAngle;
    }

    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public double getposX() {
        return posX;
    }

    public double getposY() {
        return posY;
    }

    public double adjust(double number, double min, double max) {
        if (number < min) {
            number = min;
        }
        if (number > max) {
            number = max;
        }
        return number;
    }
    public void runToPosition(double x, double y, double movementSpeed, double turnSpeed, double desiredAngle, double angleAdjuster, double distanceErrorMargin) {
        //double preferredAngle = 0;
        double movement_x;
        double movement_y;
        double movement_turn;

        double distanceToTarget = Math.hypot(x - getposX(), y - getposY());
        double absoluteAngleToTarget = Math.atan2(y - getposY(), x - getposX());
        //double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (getAngle() - Math.toRadians(0)));
        double relativeXToPoint = Math.cos(absoluteAngleToTarget) * distanceToTarget;
        double relativeYToPoint = Math.sin(absoluteAngleToTarget) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;
        //double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        movement_turn = adjust(-getAngle() / 10, -1.0, 1.0) * turnSpeed;
        if (Math.abs(distanceToTarget) < 10.0) {
            movement_turn = 0.0;
            movement_x = movementXPower * movementSpeed * .3;
            movement_y = movementYPower * movementSpeed * .3;
        }
        frontLeft.setPower((movement_y + movement_x + movement_turn));
        backLeft.setPower((movement_y - movement_x + movement_turn));
        frontRight.setPower((movement_y - movement_x - movement_turn));
        backRight.setPower((movement_y + movement_x - movement_turn));

        if (Math.abs(getAngle()) > angleAdjuster && (getAngle() - desiredAngle) > angleAdjuster || (getAngle() - desiredAngle) < -angleAdjuster) {
            frontLeft.setPower((movement_turn * 1.4));
            backLeft.setPower((movement_turn * 1.4));
            frontRight.setPower((-movement_turn * 1.4));
            backRight.setPower((-movement_turn * 1.4));
        }

        if (Math.abs(distanceToTarget) < distanceErrorMargin) {
            frontLeft.setPower(adjust(-(.04 * (getAngle() - desiredAngle)), -1, 1));
            backLeft.setPower(adjust(-(.04 * (getAngle() - desiredAngle)), -1, 1));
            frontRight.setPower(adjust((.04 * (getAngle() - desiredAngle)), -1, 1));
            backRight.setPower(adjust((.04 * (getAngle() - desiredAngle)), -1, 1));
        }
        if (Math.abs(distanceToTarget) < distanceErrorMargin && Math.abs(getAngle()) < .5) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            RunToPoint = true;
        }

        telemetry.addData("movement_y", movement_y);
        telemetry.addData("movement_x", movement_x);
        telemetry.addData("movement_turn", movement_turn);
        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("absoluteAngleToPoint", absoluteAngleToTarget);
        //telemetry.addData("relativeAngleToPoint", relativeAngleToPoint);
        telemetry.addData("relativeXToPoint", relativeXToPoint);
        telemetry.addData("relativeYToPoint", relativeYToPoint);
        telemetry.addData("movementXPower", movementXPower);
        telemetry.addData("movementYPower", movementYPower);
        //telemetry.addData("relativeTurnAngle", relativeTurnAngle);
    }


    /*public double getAngle() {
        updateRobot();
        return angle;
    }*/
    public double getCoordinateX() {
        updateRobot();
        return coordinateX;
    }
    public double getCoordinateY() {
        updateRobot();
        return  coordinateY;
    }
    public double getWobbleArmPosition() {
        updateRobot();
        return wobbleArmPosition;
    }
    public void updateRobot() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                DEGREES).firstAngle;
        // coordinateX = odometry.x;
        // coordinateY = odometry.y;
    }

    private void startFps() {
        startTime = System.currentTimeMillis();
        timeElapsed = startTime;
    }
    public void updateFps() {
        frameCount ++;
        if (System.currentTimeMillis() - timeElapsed > 1000) {
            fps = frameCount;
            frameCount = 0;
            timeElapsed = System.currentTimeMillis();
        }
    }
    public int getFps() {
        return fps;
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Inits and stuff
//    }

    public DcMotor fastestMotor() {
        DcMotor fastestMotor = motors[3];
        for (DcMotor motor : copyOf(motors, 3)) {
            fastestMotor = Math.abs(fastestMotor.getPower()) > Math.abs(motor.getPower()) ?
                    fastestMotor : motor;
        }
        return fastestMotor;
    }
    public void goMaxSpeed() {
        double multiplier = 1 / fastestMotor().getPower();
        for (DcMotor motor : motors) {
            motor.setPower(multiplier * motor.getPower());
        }
    } // Was throwing null object reference error
    public void brake() {
        for (DcMotor motor : motors) {
            motor.setPower(motor.getPower() * -0.16);
        }
        sleep(100);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void drive(float targetDegree, double speed) {
        double vertical = speed * Math.cos(Math.toRadians(targetDegree));
        double horizontal = speed * Math.sin(Math.toRadians(targetDegree));
        drive(vertical, horizontal);
    }
    public void drive(double vertical, double horizontal) {
        backLeft.setPower(vertical - horizontal + turn);
        backRight.setPower(vertical + horizontal - turn);
        frontRight.setPower(vertical - horizontal - turn);
        frontLeft.setPower(vertical + horizontal + turn);
    }
    public void drive(double direction, int distance) { // WIP
        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1);
        backLeft.setTargetPosition(-50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.1);
        frontRight.setTargetPosition(-50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.1);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.1);
        sleep(200);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(500);
    } // WIP
    public void turn(double speed) {
        turn = speed;
    }

    public void driveGyroStraight(double targetAngle) {
        double adjustment = targetAngle - getAngle();
        backLeft.setPower(frontRight.getPower() + adjustment);
        backRight.setPower(backRight.getPower() + adjustment);
        frontRight.setPower(frontRight.getPower() + adjustment);
        frontLeft.setPower(frontRight.getPower() + adjustment);
    }

    public void driveStraightWhileTurn(float targetDegree, double speed) {
        drive((float)(targetDegree + 90 - getAngle()), speed);
    }
    public void driveStraightWhileTurn(double vertical, double horizontal) {
        double targetDegree = Math.toDegrees(Math.atan(horizontal / vertical));
        targetDegree += vertical > 0 ? 0 : 180;
        double speed = Math.sqrt(Math.pow(vertical, 2) + Math.pow(horizontal, 2));
        driveStraightWhileTurn((float)targetDegree, speed);
    }

    public void prepareShoot() {
        frontShoot.setPower(0.75);
        backShoot.setPower(0.5);
    }
    public void shoot() {

    }

    public void grabWobbleGoal() {

    } // WIP
    public void releaseWobbleGoal() {

    } // WIP


    public void initialize() {
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontShoot = hardwareMap.dcMotor.get("frontShoot");
        backShoot = hardwareMap.dcMotor.get("backShoot");
        wobbleArm = hardwareMap.dcMotor.get("wobbleArm");
        shootServo = hardwareMap.servo.get("shootServo");
        wobbleGoalServo = hardwareMap.servo.get("wobbleGoalServo");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "imu";
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersIMU);
        composeTelemetryIMU();

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //vuforiaCode.config();
    }
    public int cameraMonitorViewId = hardwareMap.appContext.getResources().
            getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    protected final void readyToGo() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        startFps();
        vuforiaCode.targetsUltimateGoal.activate();
    }

    protected final void composeTelemetryIMU() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                imuAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                        DEGREES);
                imuGravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(imuAngles.angleUnit, imuAngles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(imuAngles.angleUnit, imuAngles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(imuAngles.angleUnit, imuAngles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return imuGravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(imuGravity.xAccel * imuGravity.xAccel
                                        + imuGravity.yAccel * imuGravity.yAccel
                                        + imuGravity.zAccel * imuGravity.zAccel));
                    }
                });
    }
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f",
                DEGREES.normalize(degrees));
    }

    protected RingDetection pipeline = new RingDetection();

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
        private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(650, 400);

        private final int REGION_WIDTH = 150;
        private final int REGION_HEIGHT = 125;

        private int FOUR_RING_THRESHOLD = 145;
        private int ONE_RING_THRESHOLD = 130;

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
        int avg1;

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


    VuforiaCode vuforiaCode = new VuforiaCode();

    public class VuforiaCode {

        private final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
        private final boolean PHONE_IS_PORTRAIT = false;

        private final String VUFORIA_KEY =
                "AYeVcF3/////AAABmeH5+BbXYEN4qpSLicoaxc+IVmD28hHtX0bxbefQtfKtYetF2JMES/y5/OB6AmulvNMR21cz8M5Sm0TaSIJ9VyiYtyQFy0DS0AwFaN5BGLmZoAIL9FVpYudirubarSUaywQcQR9eyQNIOwlmsIRBjZU5WC1vBxUcZ8em+bF4kZMxx9W41i0eMz86/+iMuBYi/+mwYVE0C4EM3NBUIw6XLwEL+hKI24ZWDd7i6uzPuw/Scgl8wbVEKxksgUTAdYQaioAVDZsqIMU33lmoOo0U2GToSTu3LHnhbJv6RFa2sUH5ndBmgqbcdDavCyiXTGaWHqDc404amMpkUYzX3ZVO0UQaYbfJbfnSDrx8I4uiL/FA";

        private final float mmPerInch        = 25.4f;
        private final float mmTargetHeight   = (6) * mmPerInch;

        private final float halfField = 72 * mmPerInch;
        private final float quadField  = 36 * mmPerInch;

        // Class Members
        private OpenGLMatrix lastLocation = null;
        private VuforiaLocalizer vuforia = null;

        private boolean targetVisible = false;
        private float phoneXRotate    = 0;
        private float phoneYRotate    = 0;
        private float phoneZRotate    = 0;

        VuforiaTrackables targetsUltimateGoal;

        {
            assert this.vuforia != null;
            targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        }

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


        private void config() {
            VuforiaLocalizer.Parameters parametersVuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parametersVuf.vuforiaLicenseKey = VUFORIA_KEY;

            parametersVuf.cameraName = webcamName;

            parametersVuf.useExtendedTracking = false;

            vuforia = ClassFactory.getInstance().createVuforia(parametersVuf);

            VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
            blueTowerGoalTarget.setName("Blue Tower Goal Target");
            VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
            redTowerGoalTarget.setName("Red Tower Goal Target");
            VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
            redAllianceTarget.setName("Red Alliance Target");
            VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
            blueAllianceTarget.setName("Blue Alliance Target");
            VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
            frontWallTarget.setName("Front Wall Target");

            allTrackables.addAll(targetsUltimateGoal);

            redAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, -halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            blueAllianceTarget.setLocation(OpenGLMatrix
                    .translation(0, halfField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            frontWallTarget.setLocation(OpenGLMatrix
                    .translation(-halfField, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

            // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
            blueTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
            redTowerGoalTarget.setLocation(OpenGLMatrix
                    .translation(halfField, -quadField, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            if (CAMERA_CHOICE == BACK) {
                phoneYRotate = -90;
            } else {
                phoneYRotate = 90;
            }

            if (PHONE_IS_PORTRAIT) {
                phoneXRotate = 90 ;
            }

            // Next, translate the camera lens to where it is on the robot.
            // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
            final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
            final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
            final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            /*  Let all the trackable listeners know where the phone is.  */
            for (VuforiaTrackable trackable : allTrackables) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersVuf.cameraDirection);
            }
        }

        public void updateVuforia() {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                vuforiaXPos = -translation.get(1) / mmPerInch + 24;
                vuforiaYPos = translation.get(0) / mmPerInch + 72;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        vuforiaXPos, vuforiaYPos, translation.get(2) / mmPerInch);

                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                vuforiaOrientation = rotation.firstAngle;
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
        }

    }

    protected double vuforiaXPos = 0;
    protected double vuforiaYPos = 0;
    protected double vuforiaOrientation = 0;

}