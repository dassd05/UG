package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "Combo", group = "Sensor")

public class Combine extends Robot {

    double globalAngle;
    Orientation lastAngles;
//    PIDController pidRotate, pidDrive;

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
/*        pidRotate = new PIDController (.003, .00003, 0);

        pidDrive = new PIDController (.05, 0, 0);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90,90);
        pidDrive.enable();
*/
        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        double Scalar_Value;
        double CurrentOrientation;
        double vertical;
        double horizontal;
        double turn;
        double TargetOrientation = 0;

        initialize();

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                vertical = -0.7 * gamepad1.right_stick_y;
                horizontal = 0.7 * gamepad1.right_stick_x;
                turn = -0.7 * gamepad1.left_stick_x;
                if (gamepad1.right_bumper) {
                    //turn = -0.6 * gamepad1.left_stick_x;
                    vertical = -1 *gamepad1.right_stick_y;
                    horizontal = 1 * gamepad1.right_stick_x;
                    if (vertical != 0 && horizontal != 0)
                        turn = -1 * gamepad1.left_stick_x;
                }
                if (vertical == 0 && horizontal == 0) {
                    turn = -0.25 * gamepad1.left_stick_x;
                    if (gamepad1.right_bumper) {
                        turn = -0.6 * gamepad1.left_stick_x;
                    }
                }
                if (turn != 0) {
                    frontRight.setPower(turn + (vertical - horizontal));
                    backRight.setPower(turn + (vertical + horizontal));
                    frontLeft.setPower(-turn + (vertical + horizontal));
                    backLeft.setPower(-(-turn + (vertical - horizontal)));
                }
/*            if (gamepad1.left_bumper) {
                resetAngle();
            }
*/          CurrentOrientation = /*pidDrive.performPID(getAngle());*/imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                Scalar_Value = -0.03 * (TargetOrientation - CurrentOrientation);

                if (turn == 0) {
                    frontRight.setPower(turn + (vertical - horizontal));
                    backRight.setPower(turn + (vertical + horizontal));
                    frontLeft.setPower(-turn + (vertical + horizontal));
                    backLeft.setPower(-(-turn + (vertical - horizontal)));
                }
                if (gamepad1.x) {
                    TargetOrientation = CurrentOrientation;
                }
                if (gamepad1.y) {
                    TargetOrientation = 0;
//                Scalar_Value = -0.03 * (TargetOrientation - CurrentOrientation);
                    frontRight.setPower(turn + (vertical - horizontal) - Scalar_Value);
                    backRight.setPower(turn + (vertical + horizontal) - Scalar_Value);
                    frontLeft.setPower(-turn + (vertical - horizontal) + Scalar_Value);
                    backLeft.setPower(-(-turn + (vertical + horizontal)+ Scalar_Value));
                    if (TargetOrientation - CurrentOrientation < 2.5 && TargetOrientation - CurrentOrientation > -2 && TargetOrientation - CurrentOrientation != 0 && vertical == 0 && horizontal == 0) {
                        Scalar_Value = -.12 * (TargetOrientation - CurrentOrientation);
                        frontRight.setPower(-Scalar_Value);
                        backRight.setPower(-Scalar_Value);
                        frontLeft.setPower(Scalar_Value);
                        backLeft.setPower(Scalar_Value);
                    } else {
                        Scalar_Value = -0.03 * (TargetOrientation - CurrentOrientation);
                    }
                }
                if (turn == 0 && gamepad1.left_bumper) {
//                resetAngle();
//                Scalar_Value = -0.03 * (TargetOrientation - CurrentOrientation);
                    frontRight.setPower(turn + (vertical - horizontal) - Scalar_Value);
                    backRight.setPower(turn + (vertical + horizontal) - Scalar_Value);
                    frontLeft.setPower(-turn + (vertical - horizontal) + Scalar_Value);
                    backLeft.setPower(-(-turn + (vertical + horizontal)+ Scalar_Value));
                    if (horizontal != 0 && vertical == 0) {
                        frontRight.setPower(turn + vertical - horizontal - Scalar_Value);
                        backRight.setPower(turn + vertical + horizontal - Scalar_Value);
                        frontLeft.setPower(-(-turn + vertical - horizontal + Scalar_Value));
                        backLeft.setPower(-turn + vertical + horizontal + Scalar_Value);
                    }
                    if (TargetOrientation - CurrentOrientation < 2.5 && TargetOrientation - CurrentOrientation > -2 && TargetOrientation - CurrentOrientation != 0 && vertical == 0 && horizontal == 0) {
                        Scalar_Value = -.12 * (TargetOrientation - CurrentOrientation);
                        frontRight.setPower(-Scalar_Value);
                        backRight.setPower(-Scalar_Value);
                        frontLeft.setPower(Scalar_Value);
                        backLeft.setPower(Scalar_Value);
                    } else {
                        Scalar_Value = -0.03 * (TargetOrientation - CurrentOrientation);
                    }
                }

/*            if (turn == 0 && horizontal > 0 && Scalar_Value < 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (turn == 0 && horizontal > 0 && Scalar_Value > 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (turn == 0 && horizontal < 0 && Scalar_Value < 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (turn == 0 && horizontal < 0 && Scalar_Value > 0) {
                frontRight.setPower(Scalar_Value + (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value + (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value - (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value - (-turn + (vertical + horizontal)));
            }
            if (vertical > 0 && turn==0 && Scalar_Value > 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (vertical > 0 && turn==0 && Scalar_Value < 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (vertical < 0 && turn==0 && Scalar_Value > 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
            if (vertical < 0 && turn==0 && Scalar_Value < 0) {
                frontRight.setPower(Scalar_Value - (turn + (vertical - horizontal)));
                backRight.setPower(Scalar_Value - (turn + (vertical + horizontal)));
                frontLeft.setPower(Scalar_Value + (-turn + (vertical - horizontal)));
                backLeft.setPower(Scalar_Value + (-turn + (vertical + horizontal)));
            }
*/              telemetry.addLine()
                        .addData("Scalar_Value", Scalar_Value
                        )
                        .addData("vertical", vertical
                        )
                        .addData("horizontal", horizontal
                        )
                        .addData("TargetOrientation", TargetOrientation
                        )
                        .addData("turn", turn
                        );
                telemetry.update();
            }

        }
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Vuforiaparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters Vuforiaparameters = new VuforiaLocalizer.Parameters();

        Vuforiaparameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        Vuforiaparameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        Vuforiaparameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(Vuforiaparameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
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

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
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

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
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

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, Vuforiaparameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        float vuforiaXPos = 0;
        float vuforiaYPos = 0;
        float vuforiaOrientation = 0;

        targetsUltimateGoal.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                vuforiaXPos = -translation.get(1) / mmPerInch + 24;
                vuforiaYPos = translation.get(0) / mmPerInch + 72;
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        vuforiaXPos, vuforiaYPos, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                vuforiaOrientation = rotation.firstAngle;
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angles.firstAngle = 0;
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry () {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
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
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle (AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees (double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AYeVcF3/////AAABmeH5+BbXYEN4qpSLicoaxc+IVmD28hHtX0bxbefQtfKtYetF2JMES/y5/OB6AmulvNMR21cz8M5Sm0TaSIJ9VyiYtyQFy0DS0AwFaN5BGLmZoAIL9FVpYudirubarSUaywQcQR9eyQNIOwlmsIRBjZU5WC1vBxUcZ8em+bF4kZMxx9W41i0eMz86/+iMuBYi/+mwYVE0C4EM3NBUIw6XLwEL+hKI24ZWDd7i6uzPuw/Scgl8wbVEKxksgUTAdYQaioAVDZsqIMU33lmoOo0U2GToSTu3LHnhbJv6RFa2sUH5ndBmgqbcdDavCyiXTGaWHqDc404amMpkUYzX3ZVO0UQaYbfJbfnSDrx8I4uiL/FA";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.6f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


}

