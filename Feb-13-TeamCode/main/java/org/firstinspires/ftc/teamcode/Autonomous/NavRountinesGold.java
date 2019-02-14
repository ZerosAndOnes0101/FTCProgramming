package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "NavRountinesGold", group = "Tests")
public class NavRountinesGold extends Nav_Routines {

    boolean goldfound = false;
    int leftcenterright = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();


        goldfound = checktfod();
        if (!goldfound) {
            go_forward(14, 0, -.4, false);
            goldfound = checktfod();
            if (!goldfound) {
                go_forward(29, 0, .4, false);
            } else {
                leftcenterright = 3;
            }
        } else {
            leftcenterright = 2;
        }
        telemetry.addData("goldfound", goldfound);
        telemetry.addData("leftcenterright", leftcenterright);
    }
}
