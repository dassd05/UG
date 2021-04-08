package org.firstinspires.ftc.teamcode.drive.testing;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Config
@Autonomous
public class PIDTest extends LinearOpMode {

    private DcMotorEx frontShoot, backShoot;

    private DcMotorEx wobbleArm;

    double integralf = 0;
    double integralb = 0;

    double integral = 0;

    public static PIDCoefficients pidConstsf = new PIDCoefficients(0.4, 0, 83);
    public static PIDCoefficients pidConstsb = new PIDCoefficients(0.4, 0, 181);

    public static PIDCoefficients pidConsts = new PIDCoefficients(0.4, 0.004, 0);

    FtcDashboard dashboard;

    boolean runShooterMotors = true;

    public static double speed = 1280;

    public static double position = 10;

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");
        frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");

        wobbleArm = hardwareMap.get(DcMotorEx.class, "wobbleArm");

        backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        while (opModeIsActive()) {

            PID(position);

            dashboardTelemetry.addData("position", wobbleArm.getCurrentPosition());
            dashboardTelemetry.addData("targetPosition", position);
            dashboardTelemetry.update();

            /*dashboardTelemetry.addData("backShoot velocity", backShoot.getVelocity());
            dashboardTelemetry.addData("frontShoot velocity", frontShoot.getVelocity());
            dashboardTelemetry.update();*/
        }

    }

    public void PID(double targetPosition) {

        double lastError = 0;
        double currentPosition = wobbleArm.getCurrentPosition();
        double error = targetPosition - currentPosition;

        while(Math.abs(error) > 1) {
            PIDTimer.reset();
            currentPosition = wobbleArm.getCurrentPosition();
            error = targetPosition - currentPosition;

            double deltaError = error - lastError;
            integral += error * PIDTimer.time();
            double derivative = -deltaError / PIDTimer.time();

            double P = pidConsts.p * error;
            double I = pidConsts.i * integral;
            double D = pidConsts.d * derivative;

            wobbleArm.setPower(P + I + D);

            lastError = error;
        }
    }

    double lastErrorf = 0;
    double lastErrorb = 0;

    public void runShooterMotors(double targetVelocity) {
        PIDTimer.reset();

            double currentVelocityf = frontShoot.getVelocity();

            double errorf = currentVelocityf - targetVelocity;

            double changeInErrorf = lastErrorf - errorf;
            integralf += -errorf * PIDTimer.time();
            double derivativef = changeInErrorf / PIDTimer.time();

            double Pf = pidConstsf.p * -errorf;
            double If = pidConstsf.i * integralf;
            double Df = pidConstsf.d * derivativef;

            frontShoot.setVelocity(Pf + If + Df + targetVelocity);

            lastErrorf = errorf;

            double currentVelocityb = backShoot.getVelocity();

            double errorb = currentVelocityb - targetVelocity;

            double changeInErrorb = lastErrorb - errorb;
            integralb += -errorb * PIDTimer.time();
            double derivativeb = changeInErrorb / PIDTimer.time();

            double Pb = pidConstsb.p * -errorb;
            double Ib = pidConstsb.i * integralb;
            double Db = pidConstsb.d * derivativeb;

            backShoot.setVelocity(Pb + Ib + Db + targetVelocity);

            lastErrorb = errorb;

           /* telemetry.addData("frontShoot current velocity", frontShoot.getVelocity());
            telemetry.addData("backShoot current velocity", backShoot.getVelocity());
            telemetry.addData("D", Db);
            telemetry.addData("time", PIDTimer.time());

            if (frontShoot.getVelocity() + backShoot.getVelocity() < (2 * speed + 35) && frontShoot.getVelocity() + backShoot.getVelocity() > (2 * speed - 35)) {
                telemetry.addData("GOOD", frontShoot.getVelocity() + backShoot.getVelocity());
            } else if (frontShoot.getVelocity() + backShoot.getVelocity() > (2 * speed + 35) || frontShoot.getVelocity() + backShoot.getVelocity() < (2 * speed - 35)) {
                telemetry.addData("BAD", frontShoot.getVelocity() + backShoot.getVelocity());
            }*/

            dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            dashboardTelemetry.addData("backShoot velocity", backShoot.getVelocity());
            dashboardTelemetry.addData("frontShoot velocity", frontShoot.getVelocity());
            dashboardTelemetry.addData("Target Velocity", speed);
            dashboardTelemetry.update();

            telemetry.update();
    }
}
