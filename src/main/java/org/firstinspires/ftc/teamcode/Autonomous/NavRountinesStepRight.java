package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepRight", group = "Tests")
public class NavRountinesStepRight extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_forward(20,-90,.4,false);// not working
        
    }
}
