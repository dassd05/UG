package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "testing")
public class ServosTest extends LinearOpMode {

    Servo liftServo, wobbleClawServo, wobbleArmServo, shootFlicker;

    @Override
    public void runOpMode() throws InterruptedException {
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        wobbleClawServo = hardwareMap.get(Servo.class, "wobbleClawServo");
        wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");
        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");
        Servo[] servos = {liftServo, wobbleClawServo, wobbleArmServo, shootFlicker};

        waitForStart();

        while (opModeIsActive()) {
            servos[0].setPosition(0);
            sleep(2000);
            servos[0].setPosition(1);
            sleep(2000);
        }
    }
}
