package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesStep2", group = "Tests")
public class NavRountinesStep2 extends Nav_Routines {
    @Override

    public void runOpMode() throws InterruptedException {
        Nav_Init();
        double power_adjustment= currentheadingreading();
        telemetry.addData("position",power_adjustment );
        go_sideways(180, 0, .5, 7); // forward
        sleep(100);

    }
}
