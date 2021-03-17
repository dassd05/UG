package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a modified SampleMecanumDrive class that implements the ability to cancel a trajectory
 * following. Essentially, it just forces the mode to IDLE.
 */
@Config
public class SampleMecanumDriveCancelable extends SampleMecanumDrive {

    public SampleMecanumDriveCancelable(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void cancelFollowing() {
        mode = Mode.IDLE;
    }
}
