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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
@TeleOp
public class PIDFTrial extends LinearOpMode {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    public static double MOTOR_GEAR_RATIO = 1;

    public static boolean RUN_USING_ENCODER = true; //think this is what we should do
    public static boolean DEFAULT_GAINS = false;

    public static double TESTING_SPEED = 0.5 * MOTOR_MAX_RPM;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(35, 0, 0, 15.7); //let's make this faster
    public static PIDFCoefficients MOTOR_VELO_PID_2 = new PIDFCoefficients(35, 0, 0, 15.7); //let's make this faster

    private double lastKp = 0.0;
    private double lastKi = 0.0;
    private double lastKd = 0.0;
    private double lastKf = getMotorVelocityF();

    private double lastKp_2 = 0.0;
    private double lastKi_2 = 0.0;
    private double lastKd_2 = 0.0;
    private double lastKf_2 = getMotorVelocityF();

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
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
        setPIDFCoefficients(backShoot, MOTOR_VELO_PID_2);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            setVelocity(frontShoot, TESTING_SPEED);
            setVelocity(backShoot, TESTING_SPEED);

            printVelocity(frontShoot, TESTING_SPEED);
            printVelocity(backShoot, TESTING_SPEED);

            if (lastKp_2 != MOTOR_VELO_PID_2.p || lastKi_2 != MOTOR_VELO_PID_2.i || lastKd_2 != MOTOR_VELO_PID_2.d || lastKf_2 != MOTOR_VELO_PID_2.f) {
                setPIDFCoefficients(backShoot, MOTOR_VELO_PID_2);

                lastKp_2 = MOTOR_VELO_PID_2.p;
                lastKi_2 = MOTOR_VELO_PID_2.i;
                lastKd_2 = MOTOR_VELO_PID_2.d;
                lastKf_2 = MOTOR_VELO_PID_2.f;
            }
            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(frontShoot, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private void printVelocity(DcMotorEx motor, double target) {
        telemetry.addData("targetVelocity", rpmToTicksPerSecond(target));

        double motorVelo = motor.getVelocity();
        telemetry.addData("velocity", motorVelo);
        telemetry.addData("error", rpmToTicksPerSecond(target) - motorVelo);

        telemetry.addData("upperBound", rpmToTicksPerSecond(TESTING_SPEED) * 1.15);
        telemetry.addData("lowerBound", 0);
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

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }
}
