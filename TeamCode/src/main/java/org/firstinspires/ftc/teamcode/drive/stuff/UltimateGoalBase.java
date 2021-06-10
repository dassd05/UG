package org.firstinspires.ftc.teamcode.drive.stuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.drive.ServoConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

import java.util.List;

@SuppressWarnings("unused")
public class UltimateGoalBase extends BaseBase {

    protected UltimateGoalBase.Parameters params;
    public SampleMecanumDriveCancelable drive;

    private int shotsQueued = 0;

    public Pose2d currentPose;

    public UltimateGoalBase(UltimateGoalBase.Parameters params) {
        super(params);
        this.params = params;
//        if (!(this.params.drive instanceof MecanumDrive) && this.params.drive != null) {
//            throw new IllegalArgumentException("Currently only accepts drive of type MecanumDrive, as the others haven't been implemented yet.");
//        }

        this.params.internalInit();
        this.drive = this.params.drive;
    }


    @Override
    public Pose2d getPosition() {
        return currentPose;
    }


    public void startFieldCentricDriving(double driverAngleOffset) { // counterclockwise
        currentPose = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -params.opMode.gamepad1.left_stick_y,
                -params.opMode.gamepad1.left_stick_x
        ).rotated(-currentPose.getHeading() - Math.toRadians(270));

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX() * .7,
                        input.getY() * .7,
                        (-params.opMode.gamepad1.right_stick_x * .7)
                )
        );
    }


    public void dropIntake() {
        params.droptakeStopper.setPosition(ServoConstants.dropTakeDown);
    }
    public void startIntake() {
        // todo adjust as necessary
        params.intake.setPower(1);
        params.bottomRoller.setPower(1);
    }
    public void stopIntake() {
        params.intake.setPower(0);
        params.bottomRoller.setPower(0);
    }

    public void wobbleUp() {
        double pos = params.wobbleArm1.getPosition();
        while (pos > ServoConstants.wobbleArmUp) {
            pos -= 0.01;
            params.wobbleArm1.setPosition(pos);
            params.wobbleArm2.setPosition(pos);
            sleep(20);
        }
    }
    public void wobbleDown() {
        double pos = params.wobbleArm1.getPosition();
        while (pos < ServoConstants.wobbleArmDown) {
            pos += 0.01;
            params.wobbleArm1.setPosition(pos);
            params.wobbleArm2.setPosition(pos);
            sleep(20);
        }
    }
    public void grabWobble() {
        params.wobbleClaw.setPosition(ServoConstants.wobbleClawClose);
    }
    public void releaseWobble() {
        params.wobbleClaw.setPosition(ServoConstants.wobbleClawOpen);
    }

    public void startShooter() { //Vector3D target) {
        //todo
    }
    public void stopShooter() {
        params.shooter1.setPower(0);
        params.shooter2.setPower(0);

        params.shooterStopper.setPosition(ServoConstants.shootStopperUp);
    }

    /*
        This is able to asynchronously queue shots. So if you queueShot x times rapidly,
        it will shoot x times successively, even if it's still shooting
    */
    protected Thread shoot = new Thread(() -> {
        try {
            params.shootFlicker.setPosition(ServoConstants.shootFlickerShot);
            Thread.sleep(280);
            params.shootFlicker.setPosition(ServoConstants.shootFlickerOut);
            Thread.sleep(280);
        } catch (InterruptedException e) {
            params.shootFlicker.setPosition(ServoConstants.shootFlickerOut);
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

        params.telemetry.update();

        for (LynxModule module : params.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }

    public void sleep(long ms) {
        sleep(ms, 0);
    }
    public void sleep(long ms, int nanos) {
        try {
            Thread.sleep(ms, nanos);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

//    public static double rpmToTicksPerSecond(double rpm) {
//        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
//    }
//
//    private void setVelocity(DcMotorEx motor, double power) {
//        if(RUN_USING_ENCODER) {
//            motor.setVelocity(rpmToTicksPerSecond(power));
//            Log.i("mode", "setting velocity");
//        }
//        else {
//            Log.i("mode", "setting power");
//            motor.setPower(power / MOTOR_MAX_RPM);
//        }
//    }
//    private void setVelocity2(DcMotorEx motor, double power) {
//        if (RUN_USING_ENCODER) {
//            motor.setVelocity(rpmToTicksPerSecond(power));
//            Log.i("mode", "setting velocity");
//        } else {
//            Log.i("mode", "setting power");
//            motor.setPower(power / MOTOR_MAX_RPM);
//        }
//    }
//
//    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
//        if(!RUN_USING_ENCODER) {
//            Log.i("config", "skipping RUE");
//            return;
//        }
//
//        if (!DEFAULT_GAINS) {
//            Log.i("config", "setting custom gains");
//            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
//                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
//            ));
//        } else {
//            Log.i("config", "setting default gains");
//        }
//    }


    @SuppressWarnings("unused")
    public static class Parameters extends BaseBase.Parameters {

        public SampleMecanumDriveCancelable drive;
        public LinearOpMode opMode;
        
        public List<DcMotor> shooters;
        public DcMotorEx shooter1;
        public DcMotorEx shooter2;
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


        public Parameters(OpMode opMode) {
            this(opMode, true);
        }

        public Parameters(OpMode opMode, boolean hardwareNullable) {
            super(opMode, hardwareNullable);
        }


        private void internalInit() {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            if (voltageSensor == null) voltageSensor = hardwareMap.voltageSensor.iterator().next();

            if (drive == null) drive = new SampleMecanumDriveCancelable(hardwareMap);

            shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter1.getMotorType().setAchieveableMaxRPMFraction(1.0);
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.getMotorType().setAchieveableMaxRPMFraction(1.0);
//            if (RUN_USING_ENCODER) {
//                frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            } else {
//                frontShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                backShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            }
//            setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);
//            setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

            intake = hardwareMap.dcMotor.get("intake");
            bottomRoller = hardwareMap.dcMotor.get("bottomRoller");

            turret = hardwareMap.get(Servo.class, "turret");
            flap = hardwareMap.get(Servo.class, "flap");
            wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
            wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
            shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
            droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
            wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
            shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");

            dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
            telemetry.update();
            telemetry.clearAll();
        }
    }


    public static class Hardware {

        public HardwareMap hardwareMap;

        public DcMotor frontLeft;
        public DcMotor backLeft;
        public DcMotor frontRight;
        public DcMotor backRight;

        public DcMotorEx shooter1;
        public DcMotorEx shooter2;
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
        public Camera camera;


        public Hardware(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
        }



        public void assertHardware() {

        }

        public void getMotors() {

        }

        public void getServos() {
            turret = hardwareMap.get(Servo.class, "turret");
            flap = hardwareMap.get(Servo.class, "flap");
            wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
            wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
            shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
            droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
            wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
            shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");
        }
    }
}
