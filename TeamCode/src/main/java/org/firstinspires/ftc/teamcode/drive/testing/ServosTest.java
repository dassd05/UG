package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    Servo shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw;


    public static double droptakeStopperVal = -1;
    public static double wobbleClawVal = -1;
    public static double wobbleArm1Val = -1;
    public static double wobbleArm2Val = -1;
    public static double shootFlickerVal = -1;
    public static double shooterStopperVal = -1;
    public static double flapVal = -1;
    public static double turretVal = -1;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(Servo.class, "turret");
        flap = hardwareMap.get(Servo.class, "flap");
        wobbleArm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        wobbleArm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");

        Servo[] servos = {shooterStopper, wobbleArm1, wobbleArm2, shootFlicker, flap, turret, droptakeStopper, wobbleClaw};

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            if (flapVal != -1) flap.setPosition(flapVal);
            if (wobbleArm1Val != -1) wobbleArm1.setPosition(wobbleArm1Val);
            if (wobbleArm2Val != -1) wobbleArm2.setPosition(wobbleArm2Val);
            if (shootFlickerVal != -1) shootFlicker.setPosition(shootFlickerVal);
            if (turretVal != -1) turret.setPosition(turretVal);
            if (shooterStopperVal != -1) shooterStopper.setPosition(shooterStopperVal);
            if (droptakeStopperVal != -1) droptakeStopper.setPosition(droptakeStopperVal);
            if (wobbleClawVal != -1) wobbleClaw.setPosition(wobbleClawVal);

            telemetry.update();
        }
    }
}
