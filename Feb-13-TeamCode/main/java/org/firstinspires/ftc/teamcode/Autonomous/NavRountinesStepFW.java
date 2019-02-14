package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepFW", group = "Tests")
public class NavRountinesStepFW extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
      //  go_sideways(180, 0, .5, 8); // forward
        go_forward(30, 0, .35, false);

    }
}
