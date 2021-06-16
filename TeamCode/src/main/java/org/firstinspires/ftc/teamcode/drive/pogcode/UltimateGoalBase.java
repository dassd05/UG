package org.firstinspires.ftc.teamcode.drive.pogcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.ServoConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@SuppressWarnings("unused")
public class UltimateGoalBase extends BaseBase {

    // https://github.com/ftc-9915/FtcRobotController
    // https://github.com/ftc-9915/FtcRobotController/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Vision/BlueGoalVisionPipeline.java

    // https://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/game-manual-part-2-traditional-events.pdf


    protected UltimateGoalBase.Parameters params;
    public Hardware hardware;
    public SampleMecanumDriveCancelable drive;

    private int shotsQueued = 0;
    private boolean wobbleBusy = false;

    private Pose2d position;
    private double angle;

    public UltimateGoalBase(UltimateGoalBase.Parameters params) {
        super(params);
        this.params = params;
//        if (!(this.params.drive instanceof MecanumDrive) && this.params.drive != null) {
//            throw new IllegalArgumentException("Currently only accepts drive of type MecanumDrive, as the others haven't been implemented yet.");
//        }

        this.hardware = this.params.hardware;
        this.drive = this.hardware.drive;
    }


    @Override
    public Pose2d getPosition() {
        return position;
    }
    public double getAngle() {
        return angle;
    }


    public void init() {
        params.opMode.telemetry.update();
        params.opMode.telemetry.clearAll();

        hardware.wobbleClaw.setPosition(ServoConstants.wobbleClawOpen);
        hardware.wobbleArm1.setPosition(ServoConstants.wobbleArmRest);
        hardware.wobbleArm2.setPosition(ServoConstants.wobbleArmRest);

        hardware.turret.setPosition(ServoConstants.turretShoot);
        hardware.flap.setPosition(ServoConstants.flapHG);

        hardware.shootFlicker.setPosition(ServoConstants.shootFlickerOut);

        hardware.droptakeStopper.setPosition(ServoConstants.dropTakeUp);
        hardware.shooterStopper.setPosition(ServoConstants.shootStopperUp);

        sleep(5000);

        hardware.wobbleClaw.setPosition(ServoConstants.wobbleClawClose);
    }
    public void update() {
        if (!shoot.isAlive() && shotsQueued > 0) {
            shotsQueued --;
            shoot.start();
        }

        params.telemetry.update();

        for (LynxModule module : params.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
        angle = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }


    public void startFieldCentricDriving(double driverAngleOffset) { // counterclockwise
        position = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -params.opMode.gamepad1.left_stick_y,
                -params.opMode.gamepad1.left_stick_x
        ).rotated(-position.getHeading() - Math.toRadians(270));

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * .7,
                        input.getY() * .7,
                        (-params.opMode.gamepad1.right_stick_x * .7)
                )
        );
    }


    public void dropIntake() {
        hardware.droptakeStopper.setPosition(ServoConstants.dropTakeDown);
    }
    public void startIntake() {
        // todo adjust as necessary
        hardware.intake.setPower(1);
        hardware.bottomRoller.setPower(1);
    }
    public void stopIntake() {
        hardware.intake.setPower(0);
        hardware.bottomRoller.setPower(0);
    }

    protected Thread wobbleUp = new Thread(() -> {
        double pos = hardware.wobbleArm1.getPosition();
        while (pos > ServoConstants.wobbleArmUp) {
            pos -= 0.01;
            hardware.wobbleArm1.setPosition(pos);
            hardware.wobbleArm2.setPosition(pos);
            sleep(20);
        }
        wobbleBusy = false;
    });
    protected Thread wobbleDown = new Thread(() -> {
        double pos = hardware.wobbleArm1.getPosition();
        while (pos < ServoConstants.wobbleArmDown) {
            pos += 0.01;
            hardware.wobbleArm1.setPosition(pos);
            hardware.wobbleArm2.setPosition(pos);
            sleep(20);
        }
        wobbleBusy = false;
    });
    public void wobbleUp() {
        if (wobbleBusy) {
            wobbleUp.interrupt();
            wobbleDown.interrupt();
        }
        wobbleBusy = true;
        wobbleUp.start();
    }
    public void wobbleDown() {
        if (wobbleBusy) {
            wobbleUp.interrupt();
            wobbleDown.interrupt();
        }
        wobbleBusy = true;
        wobbleDown.start();
    }
    public void grabWobble() {
        hardware.wobbleClaw.setPosition(ServoConstants.wobbleClawClose);
    }
    public void releaseWobble() {
        hardware.wobbleClaw.setPosition(ServoConstants.wobbleClawOpen);
    }

    public void startShooter(int velocity) { //Vector3D target) {
        hardware.shooter.setVelocity(velocity);
    }
    public void stopShooter() {
        hardware.shooter1.setPower(0);
        hardware.shooter2.setPower(0);

        hardware.shooterStopper.setPosition(ServoConstants.shootStopperUp);
    }

    /*
        This is able to asynchronously queue shots. So if you queueShot x times rapidly,
        it will shoot x times successively, even if it's still shooting
    */
    protected Thread shoot = new Thread(() -> {
        hardware.shootFlicker.setPosition(ServoConstants.shootFlickerShot);
        sleep(280);
        hardware.shootFlicker.setPosition(ServoConstants.shootFlickerOut);
        sleep(280);
    });
    public void queueShot() {
        shotsQueued ++;
    }

    public static void sleep(long ms) {
        sleep(ms, 0);
    }
    public static void sleep(long ms, int nanos) {
        try {
            Thread.sleep(ms, nanos);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    @SuppressWarnings("unused")
    public static class Parameters extends BaseBase.Parameters {

        public LinearOpMode opMode;
        public Hardware hardware;

        public Parameters(OpMode opMode, Hardware hardware) {
            super(opMode);
            this.hardware = hardware;
        }
    }


    public static class Hardware {

        public HardwareMap hardwareMap;

        public DcMotor frontLeft;
        public DcMotor backLeft;
        public DcMotor frontRight;
        public DcMotor backRight;
        public SampleMecanumDriveCancelable drive;

        public DcMotorEx shooter1;
        public DcMotorEx shooter2;
        public Shooter shooter;
        public DcMotor intake;
        public DcMotor bottomRoller;

        public Servo turret;
        public Servo flap;
        public Servo wobbleArm1;
        public Servo wobbleArm2;
        public Servo shootFlicker;
        public Servo droptakeStopper;
        public Servo wobbleClaw;
        public Servo shooterStopper;

        public VoltageSensor voltageSensor;
        public BNO055IMU imu;

        public OpenCvCamera webcam1;
        public OpenCvCamera webcam2;


        public Hardware(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }


        public void getAll() {
            getMotors();
            getServos();
        }
        public void getMotors() {
            getDrive();
            getIntakeMotors();
            getShooterMotors();
        }
        public void getServos() {
            getShooterServos();
            getIntakeServos();
            getWobble();
        }

        public void getDrive() {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

            drive = new SampleMecanumDriveCancelable(hardwareMap);
        }

        public void getIntake() {
            getIntakeMotors();
            getIntakeServos();
        }
        public void getIntakeMotors() {
            intake = hardwareMap.dcMotor.get("intake");
            bottomRoller = hardwareMap.dcMotor.get("bottomRoller");
        }
        public void getIntakeServos() {
            droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        }

        public void getShooter() {
            getShooterMotors();
            getShooterServos();
        }
        public void getShooterMotors() {
            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

            shooter = new Shooter(shooter1, shooter2);
        }
        public void getShooterServos() {
            turret = hardwareMap.get(Servo.class, "turret");
            flap = hardwareMap.get(Servo.class, "flap");
            shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
            shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");
        }

        public void getWobble() {
            wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
            wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
            wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        }

        public void getCameras(OpenCvPipeline pipeline1, OpenCvPipeline pipeline2) {
            getCamera1(pipeline1);
            getCamera2(pipeline2);
        }
        public void getCamera1(OpenCvPipeline pipeline) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam1.setPipeline(pipeline);

            webcam1.openCameraDeviceAsync(() -> {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam1, 0);
            });
        }
        public void getCamera2(OpenCvPipeline pipeline) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
            webcam2.setPipeline(pipeline);

            webcam2.openCameraDeviceAsync(() -> {
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam2, 0);
            });
        }

        public void getVoltageSensor() {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        public void getIMU() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }
    }
}
