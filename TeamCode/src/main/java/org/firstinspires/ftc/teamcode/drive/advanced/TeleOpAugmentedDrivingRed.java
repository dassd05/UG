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
public class TeleOpAugmentedDrivingRed extends LinearOpMode {
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


    private Servo shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw;

    private DcMotor intake, bottomRoller;


    /********************************************************************************************
     *
     */

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 17;
    public static double lastKf_2 = 17; // fix this

    /********************************************************************************************
     *
     */

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


        setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);


        setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        turret = hardwareMap.get(Servo.class, "turret");
        flap = hardwareMap.get(Servo.class, "flap");
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");

        intake = hardwareMap.dcMotor.get("intake");
        bottomRoller = hardwareMap.dcMotor.get("bottomRoller");

        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();


        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean intakeOn = false;
        boolean reversed = false;

        double reverse = 1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.left_bumper && reversed) {
                reversed = false;
            } else if (gamepad2.left_bumper && !reversed) {
                reversed = true;
            }

            if (reversed) {
                reverse = 1;
            } else if (!reversed) {
                reverse = -1;
            }

            if (gamepad2.right_bumper && intakeOn) {
                intakeOn = false;
            } else if (gamepad2.right_bumper && !intakeOn) {
                intakeOn = true;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            if (intakeOn) {
                intake.setPower(.8 * reverse);
                bottomRoller.setPower(-.7 * reverse);
            } else if (!intakeOn) {
                intake.setPower(0);
                bottomRoller.setPower(0);
            }

            Vector2d input = new Vector2d(
                    -gamepad1.right_stick_y,
                    gamepad1.right_stick_x
            ).rotated(-poseEstimate.getHeading() - Math.toRadians(270));

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * .7,
                            input.getY() * .7,
                            (-gamepad1.left_stick_x * .7)
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
        shootFlicker.setPosition(0.35);
        sleep(280);
        shootFlicker.setPosition(0.57);
    }
    public void wobbleUp () {
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(700);
        wobbleArm1.setPosition(.2);
        wobbleArm2.setPosition(.2);
        sleep(500);
    }
    public void wobbleDown () {
        wobbleArm1.setPosition(.54);
        wobbleArm2.setPosition(.54);
        sleep(500);
        //wobbleClaw.setPosition(.07); // need to change position and time
        sleep(350);
    }

    public void wobbleDeploy() {

    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    private void setVelocity(DcMotorEx motor, double power) {
        if(RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / MOTOR_MAX_RPM);
        }
    }
    private void setVelocity2(DcMotorEx motor, double power) {
        if (RUN_USING_ENCODER) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        } else {
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
}

