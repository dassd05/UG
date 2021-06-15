package org.firstinspires.ftc.teamcode.drive.stuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    
    public DcMotorEx motor1;
    public DcMotorEx motor2;

    public int ticksPerRev = 28;
    public int maxRpm = 5400;
    public double gearRatio = 1;

    public boolean runUsingEncoder = true;
    public boolean defaultGains = false;

    
    public Shooter(DcMotorEx motor1, DcMotorEx motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

        this.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor1.getMotorType().setAchieveableMaxRPMFraction(1.0);
        this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor2.getMotorType().setAchieveableMaxRPMFraction(1.0);
        this.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setVelocity(double rpm) {
        if(runUsingEncoder) {
            motor1.setVelocity(rpmToTicksPerSecond(rpm));
            motor2.setVelocity(rpmToTicksPerSecond(rpm));
        }
        else {
            motor1.setPower(rpm / maxRpm);
            motor2.setPower(rpm / maxRpm);
        }
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients, double voltage) {
        if(runUsingEncoder && !defaultGains) {
            // yk that theres a current sensor for each motor, which might be useful
            coefficients.f *= 12 / voltage;
            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
            motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }
    }

    public double rpmToTicksPerSecond(double rpm) {
        return rpm * ticksPerRev / gearRatio / 60;
    }
}
