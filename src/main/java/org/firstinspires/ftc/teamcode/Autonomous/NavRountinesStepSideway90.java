package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepSideway90", group = "Tests")
public class NavRountinesStepSideway90 extends Nav_Routines {
    @Override

    public void runOpMode() throws InterruptedException {
        Nav_Init();
        double power_adjustment= currentheadingreading();
        telemetry.addData("position",power_adjustment );
        go_sideways(90, 0, 1, 20);

    }
}
