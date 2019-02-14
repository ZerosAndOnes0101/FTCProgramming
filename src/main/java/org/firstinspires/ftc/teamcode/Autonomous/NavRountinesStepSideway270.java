package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepSideway270", group = "Tests")
public class NavRountinesStepSideway270 extends Nav_Routines {
    @Override

    public void runOpMode() throws InterruptedException {
        Nav_Init();
        double power_adjustment= currentheadingreading();
        telemetry.addData("position",power_adjustment );
       // go_sideways(270, -90, 1, 20); // back ward
        go_sideways(270, 0, .4, 10);

    }
}
