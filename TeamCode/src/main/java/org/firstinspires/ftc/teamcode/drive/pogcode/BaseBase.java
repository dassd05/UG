package org.firstinspires.ftc.teamcode.drive.pogcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public abstract class BaseBase {

    protected Parameters params;

    public BaseBase(Parameters params) {
        this.params = params.clone();
    }


    public Parameters getParameters() {
        return this.params;
    }

    public abstract Pose2d getPosition();


    public static class Parameters implements Cloneable {
        public HardwareMap hardwareMap;
        public OpMode opMode;
        public Telemetry telemetry;

        public FtcDashboard dashboard;

        public Parameters(OpMode opMode) {
            this.opMode = opMode;
            this.hardwareMap = opMode.hardwareMap;
            this.telemetry = opMode.telemetry;
        }

        @NotNull @NonNull
        public Parameters clone() {
            try {
                return (Parameters) super.clone();
            } catch (CloneNotSupportedException e) {
                throw new RuntimeException(Arrays.toString(e.getStackTrace()) + "\nParameters unable to be cloned");
            }
        }
    }
}
