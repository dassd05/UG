package org.firstinspires.ftc.teamcode.drive.advanced;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.*;


@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    enum Mode {
        INTAKING,
        TELEOP_SHOOTING,
        ENDGAME
    }

    Mode currentMode = Mode.INTAKING;

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER = true;
    public static boolean DEFAULT_GAINS = false;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    //private DcMotorEx frontShoot, backShoot;
    private Servo wobbleClawServo, wobbleArmServo;
    private Servo /*liftServo,*/ shootFlicker;
    //private DcMotor intake1, intake2;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 16.7;
    public static double lastKf_2 = 16.7; // fix this

    double lastVoltage = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        DcMotorEx frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");
        frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType = frontShoot.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        frontShoot.setMotorType(motorConfigurationType);

        DcMotorEx backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");
        backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        backShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType2 = backShoot.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        backShoot.setMotorType(motorConfigurationType2);

        if (RUN_USING_ENCODER)
            frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            frontShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (RUN_USING_ENCODER)
            backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            backShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        /*frontShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));*/
        setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

        /*backShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID_2.p, MOTOR_VELO_PID_2.i, MOTOR_VELO_PID_2.d,
                MOTOR_VELO_PID_2.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));*/
        setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();

        /*intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        liftServo = hardwareMap.servo.get("liftServo");*/
        wobbleClawServo = hardwareMap.servo.get("wobbleClawServo");
        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        shootFlicker = hardwareMap.servo.get("shootFlicker");

        //intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading() - Math.toRadians(270));

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * .7,
                            input.getY() * .7,
                            (-gamepad1.right_stick_x * .7)
                    )
            );

            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            switch (currentMode) {
                case INTAKING:
                    frontShoot.setPower(0);
                    backShoot.setPower(0);

                    /*liftServo.setPosition(.72);

                    intake1.setPower(.75);
                    intake2.setPower(.75);*/
                    break;
                case TELEOP_SHOOTING:
                    if (gamepad1.x && drive.isBusy() || gamepad1.a && drive.isBusy())
                        drive.cancelFollowing();

                    Trajectory shooting = drive.trajectoryBuilder(poseEstimate)
                            .lineToLinearHeading(new Pose2d(-10, -7, Math.toRadians(3))) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                                /*intake1.setPower(0);
                                intake2.setPower(0);
                                liftServo.setPosition(.17);*/

                                //runShooterMotors(2700); //need to test shooter power stuff
                            })
                            .build(); //need to fix the coordinates

                    drive.followTrajectory(shooting);
                    //runShooterMotors(1200);
                    break;
                case ENDGAME:
                    if (gamepad1.x && drive.isBusy() || gamepad1.y && drive.isBusy())
                        drive.cancelFollowing();

                    Trajectory endgame1 = drive.trajectoryBuilder(poseEstimate)
                            .strafeRight(10) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                                /*intake1.setPower(0);
                                intake2.setPower(0);
                                liftServo.setPosition(.17);*/

                                // runShooterMotors(2800); //need to test shooter power stuff
                            })
                            .build(); //need to fix the coordinates

                    Trajectory endgame2 = drive.trajectoryBuilder(endgame1.end())
                            .strafeRight(8) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                            })
                            .build(); //need to fix the coordinates

                    Trajectory endgame3 = drive.trajectoryBuilder(endgame2.end())
                            .strafeRight(8) // need to fix coordinate
                            .addTemporalMarker(0, () -> {
                            })
                            .build(); //need to fix the coordinates

                    drive.followTrajectory(endgame1);
                    sleep(100);
                    shoot();
                    sleep(200);

                    drive.followTrajectory(endgame2);
                    sleep(100);
                    shoot();
                    sleep(200);

                    drive.followTrajectory(endgame3);
                    sleep(100);
                    shoot();
                    sleep(200);

                    currentMode = Mode.INTAKING;
                    break;
                default:
                    // this should *NEVER* happen
                    String errMsg = "LINE 49, 126, 135, switch statement on currentMode reached default case somehow";
                    telemetry.addData("ERROR", errMsg);
                    telemetry.update();
                    throw new InternalError(errMsg);
            }

            if (lastKf_2 != MOTOR_VELO_PID_2.f) {
                MOTOR_VELO_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
                lastKf_2 = MOTOR_VELO_PID_2.f;
            }

            if (lastKf != MOTOR_VELO_PID.f) {
                MOTOR_VELO_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
                lastKf = MOTOR_VELO_PID.f;
            }

            setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);
            setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

            lastVoltage = batteryVoltageSensor.getVoltage();

            if (gamepad1.x) currentMode = Mode.INTAKING;
            if (gamepad1.y) currentMode = Mode.TELEOP_SHOOTING;
            if (gamepad1.a) currentMode = Mode.ENDGAME;

            if (gamepad1.b) {
                shoot();
            }

            // lifting wobble goal
            if (gamepad1.dpad_up) {
                wobbleUp();
            }

            // setting arm down
            if (gamepad1.dpad_down) {
                wobbleDown();
            }

            // dropping off wobble goal
            if (gamepad1.dpad_left) {
                wobbleDeploy();
            }
        }

            telemetry.update();
        }

    public void shoot() {
        shootFlicker.setPosition(0.4);
        sleep(100);
        shootFlicker.setPosition(0.1);
    }
    public void wobbleUp () {
        wobbleClawServo.setPosition(.07); // need to change position and time
        sleep(700);
        wobbleArmServo.setPosition(.8);
    }
    public void wobbleDown () {
        wobbleArmServo.setPosition(.44);
        wobbleClawServo.setPosition(.51); //need to change position and time
        sleep(1200);
    }
    public void wobbleDeploy () {
        wobbleArmServo.setPosition(.7);
        wobbleClawServo.setPosition(.51); //need to change position and time
        sleep(1200);
    }

    public void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }
    private void setPIDFCoefficients2(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!RUN_USING_ENCODER) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!DEFAULT_GAINS) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}

