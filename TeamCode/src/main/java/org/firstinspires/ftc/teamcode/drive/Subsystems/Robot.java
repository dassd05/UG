package org.firstinspires.ftc.teamcode.drive.Subsystems;

import android.util.Log;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.teamcode.drive.OtherConstants.*;
import static org.firstinspires.ftc.teamcode.drive.ServoConstants.*;

public class Robot {
    public Servo wobbleArm1 = null, wobbleArm2 = null, flap = null, turret = null,
            shooterStopper = null, shootFlicker = null, droptakeStopper = null, wobbleClaw = null;

    public DcMotor intake = null, bottomRoller = null;

    public DcMotor frontLeft = null, backRight = null, backLeft = null, frontRight = null;

    public DcMotorEx shooter1 = null, shooter2 = null;

    public VoltageSensor batteryVoltageSensor;

    public BNO055IMU imu;

    HardwareMap hwMap = null;

    public Robot(){

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

        setPIDFCoefficients(shooter1, MOTOR_VELO_PID);


        setPIDFCoefficients2(shooter2, MOTOR_VELO_PID_2);
    }

    public enum wobbleGoalState {
        DOWN,
        LIFT,
        DEPLOY,
        STOW
    }

    public wobbleGoalState wgState = wobbleGoalState.STOW;

    public ElapsedTime wgTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void updateWGState() {
        switch (wgState) {
            case DOWN:
                wobbleClaw.setPosition(wobbleClawOpen);

                wgTimer.reset();

                if (wgTimer.time() > 100) {
                    wobbleArm1.setPosition(wobbleArmDown);
                    wobbleArm2.setPosition(wobbleArmDown);
                }
                break;

            case LIFT:
                wobbleClaw.setPosition(wobbleClawClose);

                wgTimer.reset();

                if (wgTimer.time() > wobbleClawClampRecovery) {
                    wobbleArm1.setPosition(wobbleArmUp);
                    wobbleArm2.setPosition(wobbleArmUp);
                }
                break;

            case STOW:
                wobbleClaw.setPosition(wobbleClawClose);

                wgTimer.reset();

                if (wgTimer.time() > wobbleClawClampRecovery) {
                    wobbleArm1.setPosition(wobbleArmRest);
                    wobbleArm2.setPosition(wobbleArmRest);
                }
                break;

            case DEPLOY:
                wobbleClaw.setPosition(wobbleClawOpen);
                break;
        }
    }

    public void wgDown() {
        wgState = wobbleGoalState.DOWN;
    }
    public void wgStow() {
        wgState = wobbleGoalState.STOW;
    }
    public void wgLift() {
        wgState = wobbleGoalState.LIFT;
    }
    public void wgDeploy() {
        wgState = wobbleGoalState.DEPLOY;
    }


    public enum ShootState {
        FLICK,
        REST
    }

    public ShootState whatState = ShootState.REST;

    ElapsedTime shooterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void updateFlickState() {
        switch (whatState) {
            case FLICK:
                shootFlicker.setPosition(shootFlickerShot);
                shooterTimer.reset();

                if (shooterTimer.time() > flickerRecoveryTime) {
                    whatState = ShootState.REST;
                }

            case REST:
                shootFlicker.setPosition(shootFlickerOut);
        }
    }

    public void flick() {
        whatState = ShootState.FLICK;
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
}
