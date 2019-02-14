package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Nav_Routines;

@Autonomous (name = "Find Line Test", group = "Tests")
public class FindLineTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        wallfollow(120, 0, .3, 16, false,  true);
    }
}
