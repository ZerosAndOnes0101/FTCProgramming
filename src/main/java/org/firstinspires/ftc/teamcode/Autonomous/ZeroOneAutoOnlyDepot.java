package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ZerosOneAutoGoldOnlyDepot", group = "Autonomous")
public class ZeroOneAutoOnlyDepot extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchhookdown();
        go_sideways(270, 0, .4, 5);
        sleep(200);

        rightFront.setPower(-1);
        rightRear.setPower(-1);
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        sleep(400);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        go_sideways(90, 0, 1, 20); // This is working towward the right

        rightFront.setPower(-1);
        rightRear.setPower(-1);
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        sleep(700);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        boxJoint.setPower(.15);
        sleep(3000);
        boxJoint.setPower(.5);
        deactivateTfod();
    }

}