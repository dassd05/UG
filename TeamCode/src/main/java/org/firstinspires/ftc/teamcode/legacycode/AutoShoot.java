package org.firstinspires.ftc.teamcode.legacycode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoShoot", group = "")
public class AutoShoot extends Robot {


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        initialize();

/*        shootServo = hardwareMap.get(Servo.class, "shootServo");
        frontShoot = hardwareMap.get(DcMotor.class, "frontShoot");
        backShoot = hardwareMap.get(DcMotor.class, "backShoot");
*/
//         Put initialization blocks here.
        shootServo.setDirection(Servo.Direction.REVERSE);


        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                //frontShoot.setPower(.75);
                //backShoot.setPower(.5);
/*                if (gamepad1.a) {
                    frontShoot.setPower(0.75);
                    backShoot.setPower(0.5);
                    for (int i = 0; i < 10; i ++) {
                        if (!gamepad1.a) {
                            break;
                        }
                        sleep(100);
                    }
                } else {
                    frontShoot.setPower(0);
                    backShoot.setPower(0);
                }*/

                shootServo.setPosition(0);
                sleep(400);
                shootServo.setPosition(0);
                sleep(400);

                telemetry.addData("Servo Position", shootServo.getPosition());
                telemetry.update();
            }
        }
    }
}
