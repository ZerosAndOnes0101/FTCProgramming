package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepBW", group = "Tests")
public class NavRountinesStepBW extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_forward(20,0,.4,false);
    }
}
