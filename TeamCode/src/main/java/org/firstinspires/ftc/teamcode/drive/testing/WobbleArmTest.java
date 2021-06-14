package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "WobbleArmTest", group = "testing" )
public class WobbleArmTest extends LinearOpMode {

    private Servo wobbleArm1, wobbleArm2;

    @Override
    public void runOpMode() {

        wobbleArm2 = hardwareMap.servo.get("wobbleArm2");
        wobbleArm1 = hardwareMap.servo.get("wobbleArm1");


        waitForStart();

        if (opModeIsActive()) {
            while(opModeIsActive()) {
                wobbleArm1.setPosition(0);
                wobbleArm2.setPosition(0);
                sleep(3000);

                wobbleArm1.setPosition(.2);
                wobbleArm2.setPosition(.2);
                sleep(3000);
            }
        }
    }
}