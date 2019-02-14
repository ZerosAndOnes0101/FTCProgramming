package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRoutinesStepSidewaysHeading225", group = "Tests")
 public class NavRoutinesStepSidewaysHeading225 extends Nav_Routines {
    @Override

    public void runOpMode() throws InterruptedException {
        Nav_Init();
        double power_adjustment= currentheadingreading();
        telemetry.addData("position",power_adjustment );
        go_sideways(0, -25, .5, 10); // This is working towward the right
    }
}
