package org.firstinspires.ftc.teamcode.drive.stuff;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MotorEx {
    
    public DcMotorEx motor;
    public VoltageSensor voltageSensor;

    public int ticksPerRev = 28;
    public int maxRpm = 5400;
    public double gearRatio = 1;

    public boolean runUsingEncoder = true;
    public boolean defaultGains = false;
    
    
    public MotorEx(DcMotorEx motor, VoltageSensor voltageSensor) {
        this.motor = motor;
        this.voltageSensor = voltageSensor;
    }


    public void setVelocity(DcMotorEx motor, double power) {
        if(runUsingEncoder) {
            motor.setVelocity(rpmToTicksPerSecond(power));
            Log.i("mode", "setting velocity");
        }
        else {
            Log.i("mode", "setting power");
            motor.setPower(power / maxRpm);
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        if(!runUsingEncoder) {
            Log.i("config", "skipping RUE");
            return;
        }

        if (!defaultGains) {
            Log.i("config", "setting custom gains");
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                    coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / voltageSensor.getVoltage()
                    // yk that theres a current sensor for each motor, which might be useful
            ));
        } else {
            Log.i("config", "setting default gains");
        }
    }

    public double rpmToTicksPerSecond(double rpm) {
        return rpm * ticksPerRev / gearRatio / 60;
    }
}
