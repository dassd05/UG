// This simple state machine is how we should we shooting as using sleep() is bad

package org.firstinspires.ftc.teamcode.drive.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.drive.ServoConstants.*;

public class Shooter extends LinearOpMode {

    private Servo shootFlicker;

    public enum ShootState {
        FLICK,
        REST
    }

    public ShootState whatState;

    ElapsedTime myTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        shootFlicker = hardwareMap.get(Servo.class, "shootFlicker");

        switch (whatState) {
            case FLICK:
                shootFlicker.setPosition(shootFlickerShot);
                myTimer.reset();

                if (myTimer.time() > flickerRecoveryTime) {
                    whatState = ShootState.REST;
                }

            case REST:
                shootFlicker.setPosition(shootFlickerOut);
        }
    }

    public void shoot() {
        whatState = ShootState.FLICK;
    }
}
