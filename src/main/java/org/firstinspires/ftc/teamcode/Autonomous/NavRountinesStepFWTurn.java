package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStepFWTurn", group = "Tests")
public class NavRountinesStepFWTurn extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_forward(20,135,.4,false);// not working
    }
}
