package org.firstinspires.ftc.teamcode.drive.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;
import static org.firstinspires.ftc.teamcode.drive.ServoConstants.*;

public class Robot {
    public Servo wobbleArm1 = null, wobbleArm2 = null, flap = null, turret = null,
            shooterStopper = null, shootFlicker = null, droptakeStopper = null, wobbleClaw = null;

    public DcMotor intake = null, bottomRoller = null;

    public DcMotor frontLeft = null, backRight = null, backLeft = null, frontRight = null;

    public DcMotorEx shooter1 = null, shooter2 = null;

    public VoltageSensor batteryVoltageSensor = null;

    public BNO055IMU imu = null;

    HardwareMap hwMap = null;
    public Telemetry telemetry = null;

    Orientation angles = null;
    Acceleration gravity = null;

    public FtcDashboard dashboard = null;

    public Robot() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        intake = hwMap.get(DcMotor.class, "intake");
        bottomRoller = hwMap.get(DcMotor.class, "bottomRoller");

        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1 = hwMap.get(DcMotorEx.class, "shooter1");

        MotorConfigurationType motorConfigurationType = shooter1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter1.setMotorType(motorConfigurationType);

        shooter2 = hwMap.get(DcMotorEx.class, "shooter2");

        MotorConfigurationType motorConfigurationType2 = shooter2.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        shooter2.setMotorType(motorConfigurationType2);

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        turret = hwMap.get(Servo.class, "turret");
        flap = hwMap.get(Servo.class, "flap");
        wobbleArm1 = hwMap.get(Servo.class, "wobbleArm1");
        wobbleArm2 = hwMap.get(Servo.class, "wobbleArm2");
        wobbleClaw = hwMap.get(Servo.class, "wobbleClaw");
        droptakeStopper = hwMap.get(Servo.class, "droptakeStopper");
        shooterStopper = hwMap.get(Servo.class, "shooterStopper");
        shootFlicker = hwMap.get(Servo.class, "shootFlicker");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.update();
        telemetry.clearAll();

        setPIDFCoefficients(shooter1, MOTOR_VELO_PID);
        setPIDFCoefficients2(shooter2, MOTOR_VELO_PID_2);

        shootState = ShootState.REST;
        wgState = wobbleGoalState.STOW;
        inState = IntakeState.OFF;
        updateAllStatesNoShooter();

        droptakeStopper.setPosition(.25);
        shooterStopper.setPosition(.9);
    }

    public enum wobbleGoalState {
        DOWN,
        LIFT,
        DEPLOY,
        STOW,
        SLOW_MODE_DOWN,
        SLOW_MODE_STOW
    }

    public wobbleGoalState wgState;

    public ElapsedTime wgTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public boolean resetTimer = true;
    public boolean isPositionReached = false;
    public double change = 0;

    public void updateWGState() {
        switch (wgState) {
            case DOWN:
                wobbleClaw.setPosition(wobbleClawOpen);

                if (wgTimer.time() > 100) {
                    wobbleArm1.setPosition(wobbleArmDown);
                    wobbleArm2.setPosition(wobbleArmDown);
                }
                break;

            case LIFT:
                wobbleClaw.setPosition(wobbleClawClose);

                if (wgTimer.time() > wobbleClawClampRecovery) {
                    wobbleArm1.setPosition(wobbleArmUp);
                    wobbleArm2.setPosition(wobbleArmUp);
                }
                break;

            case STOW:
                wobbleClaw.setPosition(wobbleClawClose);

                if (wgTimer.time() > wobbleClawClampRecovery) {
                    wobbleArm1.setPosition(wobbleArmRest);
                    wobbleArm2.setPosition(wobbleArmRest);
                }
                break;

            case DEPLOY:
                wobbleClaw.setPosition(wobbleClawOpen);
                break;

            case SLOW_MODE_DOWN:
                makeWGSlow(wobbleArmDown, wgSlowCycleTime);

                if (isPositionReached)
                    wobbleClaw.setPosition(wobbleClawOpen);

                break;

            case SLOW_MODE_STOW:
                makeWGSlow(wobbleArmRest, wgSlowCycleTime);

                if (isPositionReached)
                    wobbleClaw.setPosition(wobbleClawClose);

                break;

            default:
        }
    }

    public void makeWGSlow(float position, long cycleTime) {
        if (!isPositionReached) {
            if (resetTimer) {
                if (wgTimer.time() > cycleTime) {
                    wgTimer.reset();
                    resetTimer = false;
                }
            }

            else {
                if (position > wobbleArm1.getPosition())
                    change = .01;
                else if (position < wobbleArm1.getPosition())
                    change = -.01;
                else
                    change = 0;

                double newPos = wobbleArm1.getPosition() + change;

                wobbleArm1.setPosition(newPos);
                wobbleArm2.setPosition(newPos);

                resetTimer = true;
            }

            if (wobbleArm1.getPosition() == position)
                isPositionReached = true;
            //for letting go and closing claw once arm movement is done
        }
    }

    public void wgDown() {
        wgState = wobbleGoalState.DOWN;
        wgTimer.reset();
    }
    public void wgStow() {
        wgState = wobbleGoalState.STOW;
        wgTimer.reset();
    }
    public void wgLift() {
        wgState = wobbleGoalState.LIFT;
        wgTimer.reset();
    }
    public void wgDeploy()  {wgState = wobbleGoalState.DEPLOY;}

    public void wgSlowDown() {
        wgState = wobbleGoalState.SLOW_MODE_DOWN;
        wgTimer.reset();
        resetTimer = false;
        isPositionReached = false;
    }

    public void wgSlowStow() {
        wgState = wobbleGoalState.STOW;
        wgTimer.reset();
        resetTimer = false;
        isPositionReached = false;
    }


    public enum ShootState {
        FLICK,
        REST
    }

    public ShootState shootState;

    public ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void updateFlickState() {
        switch (shootState) {
            case FLICK:
                shootFlicker.setPosition(shootFlickerShot);

                if (shooterTimer.time() > flickerRecoveryTime) {
                    shootState = ShootState.REST;
                }
                break;

            case REST:
                shootFlicker.setPosition(shootFlickerOut);
                break;
        }
    }

    public void flick() {
        shootState = ShootState.FLICK;
        shooterTimer.reset();
    }


    public enum IntakeState {
        ON,
        REVERSE,
        OFF
    }

    public IntakeState inState;

    public void updateIntakeState() {
        switch (inState) {
            case ON:
                intake.setPower(iOn);
                bottomRoller.setPower(brOn);
                break;
            case OFF:
                intake.setPower(iOff);
                bottomRoller.setPower(brOff);
                break;
            case REVERSE:
                intake.setPower(-iOn);
                bottomRoller.setPower(-brOn);
                break;
        }
    }

    public void intakeOn()      {inState = IntakeState.ON;}
    public void intakeOff()     {inState = IntakeState.OFF;}
    public void intakeReverse() {inState = IntakeState.REVERSE;}


    public enum ShooterState {
        HIGH,
        MIDDLE,
        OFF_MIDDLE,
        OFF_HIGH
    }

    public ShooterState whatShootState;

    //todo: change all the numbers to constants in OtherConstants later
    public void updateShooterState() {
        switch (whatShootState) {
            case HIGH:
                shooterStopper.setPosition(shootStopperDown);
                flap.setPosition(flapHG);
                turret.setPosition(.16);
                setVelocity(shooter1, 2590);
                setVelocity2(shooter2, 2590);
                break;
            case MIDDLE:
                shooterStopper.setPosition(shootStopperDown);
                flap.setPosition(flapMG);
                turret.setPosition(.2);
                setVelocity(shooter1, 2900);
                setVelocity2(shooter2, 2900);
                break;
            case OFF_HIGH:
                shooterStopper.setPosition(shootStopperUp);
                flap.setPosition(flapHG);
                turret.setPosition(.2);
                shooter1.setVelocity(shooterOff);
                shooter2.setVelocity(shooterOff);
                break;
            case OFF_MIDDLE:
                shooterStopper.setPosition(shootStopperUp);
                flap.setPosition(flapMG);
                turret.setPosition(.16);
                shooter1.setVelocity(shooterOff);
                shooter2.setVelocity(shooterOff);
                break;
            default:

        }
    }

    public void shootMiddle()   {whatShootState = ShooterState.MIDDLE;}
    public void shootHigh()     {whatShootState = ShooterState.HIGH;}
    public void offMiddle()     {whatShootState = ShooterState.OFF_MIDDLE;}
    public void offHigh()       {whatShootState = ShooterState.OFF_HIGH;}


    public void updateAllStates() {
        updateAllStatesNoShooter();
        updateShooterState();
    }

    public void updateAllStatesNoShooter() {
        updateFlickState();
        updateIntakeState();
        updateWGState();
    }

    public enum ZeroRings{
        DROP_STOPPER,
        TRAJECTORY1,
        WAIT1,
        SHOOT_1,
        SHOOT_2,
        SHOOT_3,
        TRAJECTORY2,
        TRAJECTORY3,
        WOBBLE,
        WAIT2,
        WHITE_LINE,
        CORRECTION
    }

    public static ZeroRings ZeroRingState;
    public ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public enum FourRing{
        DROP_STOPPER,
        TRAJECTORY1,
        WAIT1,
        SHOOT_1,
        SHOOT_2,
        SHOOT_3,
        KNOCK,
        GO_BACK,
        INTAKE,
        SHOOT2_1,
        SHOOT2_2,
        SHOOT2_3,
        INTAKE2,
        SHOOT3_1,
        SHOOT3_2,
        TRAJECTORY6,
        WOBBLE,
        WHITE_LINE,
        CORRECTION
    }

    public static FourRing FourRingState;

    public enum OneRing{
        DROP_STOPPER,
        TRAJECTORY1,
        WAIT,
        SHOOT_1,
        SHOOT_2,
        SHOOT_3,
        INTAKE,
        SHOOT2_1,
        SHOOT2_2,
        SHOOT2_3,
        TRAJECTORY3,
        WOBBLE,
        WHITE_LINE,
        CORRECTION
    }

    public static OneRing OneRingState;

    public void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public void setVelocity(DcMotorEx motor, double power) {
        motor.setVelocity(rpmToTicksPerSecond(power));
        Log.i("mode", "setting velocity");
    }

    public void setVelocity2(DcMotorEx motor, double power) {
        motor.setVelocity(rpmToTicksPerSecond(power));
        Log.i("mode", "setting velocity");
    }

    public void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        Log.i("config", "setting custom gains");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public void setPIDFCoefficients2(DcMotorEx motor, PIDFCoefficients coefficients) {
        Log.i("config", "setting custom gains");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }
    public void setMecanumPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void setCorrectedPIDF() {
        if (lastKf_2 != MOTOR_VELO_PID_2.f) {
            MOTOR_VELO_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
            lastKf_2 = MOTOR_VELO_PID_2.f;
        }

        if (lastKf != MOTOR_VELO_PID.f) {
            MOTOR_VELO_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
            lastKf = MOTOR_VELO_PID.f;
        }

        setPIDFCoefficients2(shooter2, MOTOR_VELO_PID_2);
        setPIDFCoefficients(shooter1, MOTOR_VELO_PID);
    }

    public void fieldCentricDrive(double driveTurn, double gamepadXCoordinate, double gamepadYCoordinate) {
        double gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
        double gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate)) + 90;
        if (gamepadDegree > 180) {
            gamepadDegree = -360 + gamepadDegree;
        }
        double robotDegree = getAngle();
        double movementRadian = Math.toRadians(gamepadDegree - robotDegree);
        double gamepadXControl = gamepadHypot * Math.cos(movementRadian);
        double gamepadYControl = gamepadHypot * Math.sin(movementRadian);

        double fr = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);
        double fl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
        double bl = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) - (gamepadXControl * Math.abs(gamepadXControl)) - driveTurn, -1, 1);
        double br = Range.clip((gamepadYControl * Math.abs(gamepadYControl)) + (gamepadXControl * Math.abs(gamepadXControl)) + driveTurn, -1, 1);

        setMecanumPowers(fl, fr, bl, br);
    }
}
