package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous
public class PIDTest extends LinearOpMode {

    private DcMotorEx frontShoot, backShoot;

    double integralf = 0;
    double integralb = 0;

    public static PIDCoefficients pidConsts = new PIDCoefficients(.1, .01, .1);

    FtcDashboard dashboard;

    boolean runShooterMotors = true;

    public static double speed = 1290;

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
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        PIDTimer.reset();
        while (opModeIsActive()) {
            runShooterMotorsPerLoop(speed);


            dashboardTelemetry.addData("backShoot velocity", backShoot.getVelocity());
            dashboardTelemetry.addData("frontShoot velocity", frontShoot.getVelocity());
            dashboardTelemetry.update();
        }

    }


    double lastErrorf = 0;
    double lastErrorb = 0;
    public void runShooterMotorsPerLoop(double targetVelocity) {
        PIDTimer.reset();

        double currentVelocityf = frontShoot.getVelocity();

        double errorf = currentVelocityf - targetVelocity;

        double changeInErrorf = lastErrorf - errorf;
        integralf += -errorf * PIDTimer.time();
        double derivativef = changeInErrorf / PIDTimer.time();

        double Pf = pidConsts.p * -errorf;
        double If = pidConsts.i * integralf;
        double Df = pidConsts.d * derivativef;

        frontShoot.setVelocity(Pf + If + Df + targetVelocity);

        lastErrorf = errorf;

        double currentVelocityb = backShoot.getVelocity();

        double errorb = currentVelocityb - targetVelocity;

        double changeInErrorb = lastErrorb - errorb;
        integralb += -errorb * PIDTimer.time();
        double derivativeb = changeInErrorb / PIDTimer.time();

        double Pb = pidConsts.p * -errorb;
        double Ib = pidConsts.i * integralb;
        double Db = pidConsts.d * derivativeb;

        backShoot.setVelocity(Pb + Ib + Db + targetVelocity);

        lastErrorb = errorb;

        telemetry.addData("frontShoot current velocity", frontShoot.getVelocity());
        telemetry.addData("backShoot current velocity", backShoot.getVelocity());
        telemetry.addData("PID", Pb + Ib + Db);

        if (frontShoot.getVelocity() + backShoot.getVelocity() < (2 * speed + 35) && frontShoot.getVelocity() + backShoot.getVelocity() > (2 * speed - 35)) {
            telemetry.addData("GOOD", frontShoot.getVelocity() + backShoot.getVelocity());
        } else if (frontShoot.getVelocity() + backShoot.getVelocity() > (2 * speed + 35) || frontShoot.getVelocity() + backShoot.getVelocity() < (2 * speed - 35)) {
            telemetry.addData("BAD", frontShoot.getVelocity() + backShoot.getVelocity());
        }

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboardTelemetry.addData("backShoot velocity", backShoot.getVelocity());
        dashboardTelemetry.addData("frontShoot velocity", frontShoot.getVelocity());
        dashboardTelemetry.update();

        telemetry.update();
    }
}
