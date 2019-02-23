package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesWinchPushIn", group = "Tests")
public class NavRountinesWinchPushIn extends Nav_Routines {
    @Override

    public void runOpMode() throws InterruptedException {
        Nav_Init();
        double power_adjustment= currentheadingreading();
        telemetry.addData("position",power_adjustment );
        winchhookup();

    }
}
