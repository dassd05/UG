package org.firstinspires.ftc.teamcode.legacycode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Config
@Autonomous
public class PIDTest extends LinearOpMode {

    DcMotor frontShoot;
    DcMotor backShoot;

    //    double integral = 0;
    double repetitions;
    //0 is placeholder all ps is and ds
    public static PIDCoefficients pidVars = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients pidConsts = new PIDCoefficients(0, 0, 0);

    //FtcDashboard dashboard;

    public static double TARGET_POS= 100;

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode(){
        backShoot = hardwareMap.dcMotor.get("backShoot");
        frontShoot = hardwareMap.dcMotor.get("frontShoot");

//        frontShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //dashboard = FtcDashboard.getInstance();

        waitForStart();

        double lastTime = System.nanoTime();
        double lastPos = frontShoot.getCurrentPosition();
        double lastPos2 = backShoot.getCurrentPosition();
        double currentVelocity;
        double currentVelocity2;

        double[] frontShootPidArr = {0, 0, 0};
        double[] backShootPidArr = {0, 0, 0};

        while (opModeIsActive()) {
            currentVelocity = (frontShoot.getCurrentPosition() - lastPos) / (System.nanoTime() - lastTime);
            frontShootPidArr = pidTest(currentVelocity, 39041, frontShootPidArr[1], frontShootPidArr[2]);
            frontShoot.setPower(frontShootPidArr[0]);
            currentVelocity2 = (backShoot.getCurrentPosition() - lastPos2) / (System.nanoTime() - lastTime);
            backShootPidArr = pidTest(currentVelocity2, 39041, backShootPidArr[1], backShootPidArr[2]);
            backShoot.setPower(backShootPidArr[0]);
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

    public double[] pidTest(double currentVelocity, double targetVelocity, double lastError, double lastIntegral) {
        double error = targetVelocity - currentVelocity;
        double changeInError = lastError - error;
        lastIntegral += changeInError * PIDTimer.time();
        double derivative = changeInError / PIDTimer.time();
        pidVars.p = pidConsts.p * error;
        pidVars.i = pidConsts.i * lastIntegral;
        pidVars.d = pidConsts.d * derivative;
        return new double[] {pidVars.p + pidVars.i + pidVars.d, error, lastIntegral}; // returns the new motor power, last error, and last integral
    }
}
