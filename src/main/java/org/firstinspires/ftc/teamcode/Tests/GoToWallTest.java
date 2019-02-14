package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Nav_Routines;

@Autonomous (name = "Go To Wall Test", group = "Tests")
public class GoToWallTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_sideways_to_wall(0,.35,16, false);
    }
}
