package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepSideway180", group = "Tests")
public class NavRountinesStepSideway180 extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_sideways(180, 0, 1, 20); // forward
    }
}
