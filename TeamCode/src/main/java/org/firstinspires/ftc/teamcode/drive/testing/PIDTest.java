package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class PIDTest extends LinearOpMode {

    DcMotorEx frontShoot;
    DcMotorEx backShoot;

    double integralf = 0;
    double integralb = 0;

    public static PIDCoefficients pidConsts = new PIDCoefficients(0, 0, 0);

    FtcDashboard dashboard;

    boolean runShooterMotors = true;

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        backShoot = hardwareMap.get(DcMotorEx.class, "backShoot");
        frontShoot = hardwareMap.get(DcMotorEx.class, "frontShoot");

        backShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            //runShooterMotors(1400);
            frontShoot.setVelocityPIDFCoefficients(1.45, .25, .4, 14);
            frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontShoot.setVelocity(1220);

            backShoot.setVelocityPIDFCoefficients(1.45, .25, .4, 14);
            backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backShoot.setVelocity(1220);

            telemetry.addData("frontShoot current velocity", frontShoot.getVelocity());
            telemetry.addData("backShoot current velocity", backShoot.getVelocity());
            telemetry.update();

        }

    }

    /*void moveTestMotor(double targetPosition){
        double error = frontShoot.getCurrentPosition();
        double lastError = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        //replace ten with motor tick count
        //30 is a place holder
        while (Math.abs(error) <= 10 && repetitions <30){
            error = frontShoot.getCurrentPosition() - targetPosition;
            double changeInError = lastError - error;
            integral += changeInError* PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = pidConsts.p * error;
            double I = pidConsts.i * integral;
            double D = pidConsts.d * derivative;
            frontShoot.setPower(P + I + D);
            lastError = error;
            repetitions++;
            PIDTimer.reset();
        }
    }*/

    /*public double[] pidTest(double currentVelocity, double targetVelocity, double lastError, double lastIntegral) {
        double error = targetVelocity - currentVelocity;
        double changeInError = lastError - error;
        lastIntegral += changeInError * PIDTimer.time();
        double derivative = changeInError / PIDTimer.time();
        pidVars.p = pidConsts.p * error;
        pidVars.i = pidConsts.i * lastIntegral;
        pidVars.d = pidConsts.d * derivative;
        return new double[] {pidVars.p + pidVars.i + pidVars.d, error, lastIntegral}; // returns the new motor power, last error, and last integral
    }*/
    public void runShooterMotors(double targetVelocity) {
        PIDTimer.reset();

        double lastErrorf = 0;

        double lastErrorb = 0;

        while (runShooterMotors) {
            double currentVelocityf = frontShoot.getVelocity();

            double errorf = currentVelocityf - targetVelocity;

            double changeInError1 = lastErrorf - errorf;
            integralf += changeInError1 * PIDTimer.time();
            double derivativef = changeInError1 / PIDTimer.time();

            double Pf = pidConsts.p * errorf;
            double If = pidConsts.i * integralf;
            double Df = pidConsts.d * derivativef;

            frontShoot.setVelocity(Pf + If + Df + targetVelocity);

            lastErrorf = errorf;

            double currentVelocityb = backShoot.getVelocity();

            double errorb = currentVelocityb - targetVelocity;

            double changeInErrorb = lastErrorb - errorb;
            integralb += changeInErrorb * PIDTimer.time();
            double derivativeb = changeInErrorb / PIDTimer.time();

            double Pb = pidConsts.p * errorb;
            double Ib = pidConsts.i * integralb;
            double Db = pidConsts.d * derivativeb;

            backShoot.setVelocity(Pb + Ib + Db + targetVelocity);

            lastErrorb = errorb;

            PIDTimer.reset();
        }
    }
}
