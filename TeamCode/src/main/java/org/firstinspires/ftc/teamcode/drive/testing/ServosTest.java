package org.firstinspires.ftc.teamcode.drive.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    Servo shooterStopper, wobbleClawServo1, wobbleClawServo2, shootFlicker, flap, turret, droptakeStopper, wobbleArm;

    public static double droptakeStopperVal = -1;
    public static double wobbleArmVal = -1;
    public static double wobbleClaw1Val = -1;
    public static double wobbleClaw2Val = -1;
    public static double shootFlickerVal = -1;
    public static double shooterStopperVal = -1;
    public static double flapVal = -1;
    public static double turretVal = -1;

    public FtcDashboard dashboard;


    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(Servo.class, "turret");
        flap = hardwareMap.get(Servo.class, "flap");
        wobbleClawServo1 = hardwareMap.get(Servo.class, "wobbleClawServo1");
        wobbleClawServo2 = hardwareMap.get(Servo.class, "wobbleClawServo2");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        droptakeStopper = hardwareMap.get(Servo.class, "droptakeStopper");
        wobbleArm = hardwareMap.get(Servo.class, "wobbleArm");
        shooterStopper = hardwareMap.get(Servo.class, "shooterStopper");

        Servo[] servos = {shooterStopper, wobbleClawServo1, wobbleClawServo2, shootFlicker, flap, turret, droptakeStopper, wobbleArm};

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            if (flapVal != -1) flap.setPosition(flapVal);
            if (wobbleClaw1Val != -1) wobbleClawServo1.setPosition(wobbleClaw1Val);
            if (wobbleClaw2Val != -1) wobbleClawServo2.setPosition(wobbleClaw2Val);
            if (shootFlickerVal != -1) shootFlicker.setPosition(shootFlickerVal);
            if (turretVal != -1) turret.setPosition(turretVal);
            if (shooterStopperVal != -1) shooterStopper.setPosition(shooterStopperVal);
            if (droptakeStopperVal != -1) droptakeStopper.setPosition(droptakeStopperVal);
            if (wobbleArmVal != -1) wobbleArm.setPosition(wobbleArmVal);

            telemetry.update();
        }
    }
}
