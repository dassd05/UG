package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "testing")
public class DashboardTest extends OpMode {
    private boolean initFirst = false;
    private boolean initLoopFirst = false;
    private boolean startFirst = false;
    private boolean loopFirst = false;

    @Override
    public void init() {
        initFirst = !initLoopFirst;
    }

    @Override
    public void init_loop() {
        initLoopFirst = !initFirst;
    }

    @Override
    public void start() {
        startFirst = !loopFirst;
    }

    @Override
    public void loop() {
        loopFirst = !startFirst;

        telemetry.addData("initFirst", initFirst);
        telemetry.addData("initLoopFirst", initLoopFirst);
        telemetry.addData("startFirst", startFirst);
        telemetry.addData("loopFirst", loopFirst);
        telemetry.update();
    }
}
