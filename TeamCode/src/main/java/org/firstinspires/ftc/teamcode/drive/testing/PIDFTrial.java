package org.firstinspires.ftc.teamcode.drive.testing;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
@TeleOp
public class  PIDFTrial extends LinearOpMode {
    //Servo shootFlicker;

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER = true;
    public static boolean DEFAULT_GAINS = false;

    public static double TESTING_SPEED = 0.5 * MOTOR_MAX_RPM;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(45, 0, 0, 25);
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(45, 0, 0, 25); // fix this

    public static double lastKf = 16.7;
    public static double lastKf_2 = 16.7; // fix this

    double lastVoltage = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontShoot = hardwareMap.get(DcMotorEx.class, "shooter1");
        //frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorConfigurationType motorConfigurationType = frontShoot.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        frontShoot.setMotorType(motorConfigurationType);

        DcMotorEx backShoot = hardwareMap.get(DcMotorEx.class, "shooter2");
        //backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
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

        //shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

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

        //shootFlicker.setPosition(.3);

        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            if (lastKf_2 != MOTOR_VELO_PID_2.f) {
                MOTOR_VELO_PID_2.f = lastKf_2 * 12 / batteryVoltageSensor.getVoltage();
                lastKf_2 = MOTOR_VELO_PID_2.f;
            }

            if (lastKf != MOTOR_VELO_PID.f) {
                MOTOR_VELO_PID.f = lastKf * 12 / batteryVoltageSensor.getVoltage();
                lastKf = MOTOR_VELO_PID.f;
            } //might need to change the way we adjust the feedforward value

            /*if (gamepad1.b) {
                shootFlicker.setPosition(.6);
                sleep(200);
                shootFlicker.setPosition(.3);
            }*/

            setPIDFCoefficients2(backShoot, MOTOR_VELO_PID_2);
            setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

            setVelocity(frontShoot, TESTING_SPEED);
            setVelocity2(backShoot, TESTING_SPEED);

            printVelocity(frontShoot, TESTING_SPEED);
            printVelocity2(backShoot, TESTING_SPEED);

            lastVoltage = batteryVoltageSensor.getVoltage();

            telemetry.addData("battery voltage", batteryVoltageSensor.getVoltage());
            telemetry.addData("feedforward", MOTOR_VELO_PID.f);
            telemetry.update();
        }
    }

    private void printVelocity(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", rpmToTicksPerSecond(target));

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", rpmToTicksPerSecond(target) - motorVelo);
    }
    private void printVelocity2(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity2", rpmToTicksPerSecond(target));

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity2", motorVelo);
        telemetry.addData("error2", rpmToTicksPerSecond(target) - motorVelo);
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
