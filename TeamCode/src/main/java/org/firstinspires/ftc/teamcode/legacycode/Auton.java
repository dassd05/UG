package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AMain", group = "sensor")
public class Auton extends Robot {

    //public double Current_Orientation;
    //public double Scalar_Value = 0;
    @Override
    public void runOpMode() {
        initialize();
        final int TICKS_PER_REV = 8192; // 8192 ticks per revolution
        final double ODO_WHEEL_CIRCUM = 35 / 25.4 * Math.PI; // Wheels are 35 mm diameter
        final double odo_constant = ODO_WHEEL_CIRCUM / TICKS_PER_REV;

        double horizontalEncoderPrevious = -backLeft.getCurrentPosition() * odo_constant;
        double verticalLeftEncoderPrevious = backRight.getCurrentPosition() * odo_constant;
        double verticalRightEncoderPrevious = -frontLeft.getCurrentPosition() * odo_constant;
        double horizontalEncoderCurrent;
        double verticalLeftEncoderCurrent;
        double verticalRightEncoderCurrent;
        double verticalEncoderCurrent;

        if (opModeIsActive()) {
            horizontalEncoderCurrent = -backLeft.getCurrentPosition() * odo_constant - horizontalEncoderPrevious;
            verticalLeftEncoderCurrent = backRight.getCurrentPosition() * odo_constant - verticalLeftEncoderPrevious;
            verticalRightEncoderCurrent = -frontLeft.getCurrentPosition() * odo_constant - verticalRightEncoderPrevious;
            verticalEncoderCurrent = (verticalLeftEncoderCurrent + verticalRightEncoderCurrent) / 2;

            if (Math.abs(getAngle() - lastAngle) > 180) {
                lastAngle += getAngle() > lastAngle ? 360 : -360;
            }
            double hAdjustment = (getAngle() - lastAngle) / 360 * 2 * 6.5 * Math.PI;
            //double hAdjustment = (verticalRightEncoderCurrent - verticalLeftEncoderCurrent);
            double h = horizontalEncoderCurrent + hAdjustment;
            double v = verticalEncoderCurrent;
            double a = Math.toRadians(getAngle());
            double d = Math.sqrt(Math.pow(h, 2) + Math.pow(v, 2));
            double b = Math.atan2(v, h) - a;
            posX += 2 * Math.cos(b) * d;
            posY += 2 * Math.sin(b) * d;

            horizontalEncoderPrevious = -backLeft.getCurrentPosition() * odo_constant;
            verticalLeftEncoderPrevious = backRight.getCurrentPosition() * odo_constant;
            verticalRightEncoderPrevious = -frontLeft.getCurrentPosition() * odo_constant;
            lastAngle = getAngle();
            while (pipeline.position == null) {}
            telemetry.addData("Number of Rings", pipeline.position);
            telemetry.update();
            switch (pipeline.position) {
                case FOUR:
                    autonFourRings();
                    break;
                case ONE:
                    autonOneRing();
                    break;
                case NONE:
                    autonZeroRings();
                    break;
                default:
                    telemetry.addData("ERROR", "NUMBER OF RINGS NOT RECOGNIZED");
                    telemetry.update();
            }
            telemetry.addData("run", "finished");
            telemetry.update();
        }
    }

    public void initialize() {
        super.initialize();

        // Does the needed adjustments of the hardware
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        readyToGo();
    }

    public void autonFourRings() {
        double Scalar_Value = 0;

        Scalar_Value = -.04 * (-getAngle());

        wobbleGoalServo.setPosition(1);
        sleep(2000);
        wobbleArm.setTargetPosition(200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.1);
        sleep(600);
        wobbleArm.setPower(0);
        sleep(300);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(-50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.1 + Scalar_Value);
        frontRight.setTargetPosition(-50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.1 + Scalar_Value);
        sleep(200);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.1) + Scalar_Value);
        frontRight.setTargetPosition(50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.1) + Scalar_Value);
        sleep(700);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        frontShoot.setPower(0.66);
        backShoot.setPower(0.5);
        sleep(2000);
        //   if (shootInitiated) {
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(750);
        frontShoot.setPower(0);
        backShoot.setPower(0);

        backRight.setTargetPosition(-150);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(-150);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.2 + Scalar_Value);
        frontRight.setTargetPosition(-150);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(-150);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.2 + Scalar_Value);
        sleep(250);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);


        backRight.setTargetPosition(-300);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(300);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.2) + Scalar_Value);
        frontRight.setTargetPosition(300);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.2 - Scalar_Value);
        frontLeft.setTargetPosition(-300);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.2) + Scalar_Value);
        sleep(620);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(300);

        backRight.setTargetPosition(-400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.25 - Scalar_Value);
        backLeft.setTargetPosition(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.25 + Scalar_Value);
        frontRight.setTargetPosition(-400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.25 - Scalar_Value);
        frontLeft.setTargetPosition(-400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.25 + Scalar_Value);
        sleep(1550);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(1000);

        wobbleArm.setTargetPosition(-200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(-0.1);
        sleep(300);
        wobbleArm.setPower(0);
        wobbleGoalServo.setPosition(0);
        sleep(300);

        backRight.setTargetPosition(650);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(.2 - Scalar_Value);
        backLeft.setTargetPosition(650);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(.2 + Scalar_Value);
        frontRight.setTargetPosition(650);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.2 - Scalar_Value);
        frontLeft.setTargetPosition(650);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(.2 + Scalar_Value);
        sleep(665);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);

    }
    public void autonOneRing() {
        double Scalar_Value = 0;
        Scalar_Value = -.04 * (-getAngle());

        wobbleGoalServo.setPosition(1);
        sleep(2000);
        wobbleArm.setTargetPosition(200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.1);
        sleep(600);
        wobbleArm.setPower(0);
        sleep(300);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(-50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.1 + Scalar_Value);
        frontRight.setTargetPosition(-50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.1 + Scalar_Value);
        sleep(200);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.1) + Scalar_Value);
        frontRight.setTargetPosition(50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.1) + Scalar_Value);
        sleep(700);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        // for (int count = 0; count < 4; count++) {
        frontShoot.setPower(0.66);
        backShoot.setPower(0.5);
        sleep(2000);
        //   if (shootInitiated) {
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(750);
        frontShoot.setPower(0);
        backShoot.setPower(0);
        //   }

        backRight.setTargetPosition(-150);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(-150);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.2 + Scalar_Value);
        frontRight.setTargetPosition(-150);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(-150);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.2 + Scalar_Value);
        sleep(250);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);


        backRight.setTargetPosition(-300);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(300);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.2) + Scalar_Value);
        frontRight.setTargetPosition(300);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.2 - Scalar_Value);
        frontLeft.setTargetPosition(-300);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.2) + Scalar_Value);
        sleep(550);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        backRight.setTargetPosition(-400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.2 + Scalar_Value);
        frontRight.setTargetPosition(-400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(-400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.2 + Scalar_Value);
        sleep(975);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(2000);

        backRight.setTargetPosition(400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(.25 - Scalar_Value);
        backLeft.setTargetPosition(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(-.25) + Scalar_Value);
        frontRight.setTargetPosition(-400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.25 - Scalar_Value);
        frontLeft.setTargetPosition(400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(.25) + Scalar_Value);
        sleep(520);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(2000);

        wobbleArm.setTargetPosition(-200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(-0.1);
        sleep(200);
        wobbleArm.setPower(0);
        wobbleGoalServo.setPosition(0);
        sleep(300);

        backRight.setTargetPosition(650);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(.2 - Scalar_Value);
        backLeft.setTargetPosition(650);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(.2 + Scalar_Value);
        frontRight.setTargetPosition(650);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.2 - Scalar_Value);
        frontLeft.setTargetPosition(650);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(.2 + Scalar_Value);
        sleep(200);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
    }
    public void autonZeroRings() {
        double Scalar_Value = 0;
        Scalar_Value = -.04 * (-getAngle());
        wobbleGoalServo.setPosition(1);
        sleep(2000);
        wobbleArm.setTargetPosition(200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.1);
        sleep(600);
        wobbleArm.setPower(0);
        sleep(300);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(-50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.1 + Scalar_Value);
        frontRight.setTargetPosition(-50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.1 + Scalar_Value);
        sleep(200);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        backRight.setTargetPosition(-50);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.1 - Scalar_Value);
        backLeft.setTargetPosition(50);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.1) + Scalar_Value);
        frontRight.setTargetPosition(50);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.1 - Scalar_Value);
        frontLeft.setTargetPosition(-50);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.1) + Scalar_Value);
        sleep(700);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        // for (int count = 0; count < 4; count++) {
        frontShoot.setPower(0.66);
        backShoot.setPower(0.5);
        sleep(2000);
        //   if (shootInitiated) {
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        //sleep(750);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(2000);
        shootServo.setPosition(0.22);
        sleep(300);
        shootServo.setPosition(-0.1);
        sleep(750);
        frontShoot.setPower(0);
        backShoot.setPower(0);


        backRight.setTargetPosition(-150);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(-150);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.2 + Scalar_Value);
        frontRight.setTargetPosition(-150);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(-150);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.2 + Scalar_Value);
        sleep(250);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);


        backRight.setTargetPosition(-300);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(300);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(.2) + Scalar_Value);
        frontRight.setTargetPosition(300);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.2 - Scalar_Value);
        frontLeft.setTargetPosition(-300);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(-.2) + Scalar_Value);
        sleep(550);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(500);

        backRight.setTargetPosition(-400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.2 - Scalar_Value);
        backLeft.setTargetPosition(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.2 + Scalar_Value);
        frontRight.setTargetPosition(-400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(-400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.2 + Scalar_Value);
        sleep(720);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(2000);


        wobbleArm.setTargetPosition(-200);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(-0.1);
        sleep(300);
        wobbleArm.setPower(0);
        wobbleGoalServo.setPosition(0);
        sleep(500);

        backRight.setTargetPosition(400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(.25 - Scalar_Value);
        backLeft.setTargetPosition(400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(.25 + Scalar_Value);
        frontRight.setTargetPosition(400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(.25 - Scalar_Value);
        frontLeft.setTargetPosition(400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(.25 + Scalar_Value);
        sleep(330);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(2000);


        backRight.setTargetPosition(350);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(.2 - Scalar_Value);
        backLeft.setTargetPosition(-350);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-(-.2) + Scalar_Value);
        frontRight.setTargetPosition(-350);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.2 - Scalar_Value);
        frontLeft.setTargetPosition(350);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-(.2) + Scalar_Value);
        sleep(480);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(100);

        backRight.setTargetPosition(-400);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(-.25 - Scalar_Value);
        backLeft.setTargetPosition(-400);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(-.25 + Scalar_Value);
        frontRight.setTargetPosition(-400);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-.25 - Scalar_Value);
        frontLeft.setTargetPosition(-400);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(-.25 + Scalar_Value);
        sleep(400);
        backRight.setPower(-Scalar_Value);
        backLeft.setPower(Scalar_Value);
        frontLeft.setPower(Scalar_Value);
        frontRight.setPower(-Scalar_Value);
        sleep(2000);
    }
}