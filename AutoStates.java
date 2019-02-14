package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoStates", group = "Autonomous")
public class AutoStates extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();

        winchhookdown();

        leftFront.setPower(-.15);
        leftRear.setPower(.15);
        rightFront.setPower(.15);
        rightRear.setPower(-.15);
        sleep(350);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
sleep(100);
        leftFront.setPower(-.15);
        leftRear.setPower(-.15);
        rightFront.setPower(-.15);
        rightRear.setPower(-.15);
        sleep(550);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(100);

        leftFront.setPower(-.15);//go right
        leftRear.setPower(.15);
        rightFront.setPower(.15);
        rightRear.setPower(-.15);
        sleep(1000);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


        goldfound = checktfod();
        if (goldfound) {
            leftFront.setPower(-.15);
            leftRear.setPower(-.15);
            rightFront.setPower(-.15);
            rightRear.setPower(-.15);
            sleep(700);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(.15);
            rightFront.setPower(.15);
            rightRear.setPower(.15);
            sleep(700);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(-.15);
            rightFront.setPower(-.15);
            rightRear.setPower(.15);
            sleep(2900);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(.15);
            rightFront.setPower(-.15);
            rightRear.setPower(-.15);
            sleep(1250);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            leftFront.setPower(-1);
            leftRear.setPower(-1);
            rightFront.setPower(-1);
            rightRear.setPower(-1);
            sleep(600);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
        }


        deactivateTfod();


    }
}
