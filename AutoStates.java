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
            sleep(400);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(-.15);
            rightFront.setPower(-.15);
            rightRear.setPower(.15);
            sleep(2650);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(.15);
            rightFront.setPower(-.15);
            rightRear.setPower(-.15);
            sleep(200);
            leftFront.setPower(-.1);
            leftRear.setPower(-.1);
            rightFront.setPower(-.1);
            rightRear.setPower(-.1);
            sleep(100);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(.15);
            rightFront.setPower(-.15);
            rightRear.setPower(-.15);
            sleep(550);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(1);
            leftRear.setPower(-1);
            rightFront.setPower(-1);
            rightRear.setPower(1);
            sleep(1300);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            minKnock.setPower(-1);
            sleep(1200);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            minKnock.setPower(0);
            sleep(100);
            leftFront.setPower(-.15);
            leftRear.setPower(-.15);
            rightFront.setPower(.15);
            rightRear.setPower(.15);
            sleep(1000);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(.15);
            leftRear.setPower(-.15);
            rightFront.setPower(-.15);
            rightRear.setPower(.15);
            sleep(1200);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(-1);
            leftRear.setPower(-1);
            rightFront.setPower(-1);
            rightRear.setPower(-1);
            sleep(2600);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);
            sleep(100);
        } else {

            leftFront.setPower(.15);
            leftRear.setPower(-.15);
            rightFront.setPower(-.15);
            rightRear.setPower(.15);
            sleep(1100);
            leftFront.setPower(0);
            leftRear.setPower(-0);
            rightFront.setPower(-0);
            rightRear.setPower(0);
            sleep(100);
            leftFront.setPower(-.15);
            leftRear.setPower(-.15);
            rightFront.setPower(.15);
            rightRear.setPower(.15);
            sleep(200);
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(-0);
            sleep(100);
            goldfound = checktfod();
            if (goldfound) {
                leftFront.setPower(-.15);
                leftRear.setPower(-.15);
                rightFront.setPower(-.15);
                rightRear.setPower(-.15);
                sleep(850);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(.15);
                leftRear.setPower(.15);
                rightFront.setPower(.15);
                rightRear.setPower(.15);
                sleep(600);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(.15);
                leftRear.setPower(-.15);
                rightFront.setPower(-.15);
                rightRear.setPower(.15);
                sleep(1800);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(.15);
                leftRear.setPower(.15);
                rightFront.setPower(-.15);
                rightRear.setPower(-.15);
                sleep(200);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(.15);
                leftRear.setPower(.15);
                rightFront.setPower(-.15);
                rightRear.setPower(-.15);
                sleep(500);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(1);
                leftRear.setPower(-1);
                rightFront.setPower(-1);
                rightRear.setPower(1);
                sleep(1200);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                minKnock.setPower(-1);
                sleep(1200);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                minKnock.setPower(0);
                sleep(100);
                //turn left
                leftFront.setPower(-.15);
                leftRear.setPower(-.15);
                rightFront.setPower(.15);
                rightRear.setPower(.15);
                sleep(800);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(.15);
                leftRear.setPower(-.15);
                rightFront.setPower(-.15);
                rightRear.setPower(.15);
                sleep(450);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
                leftFront.setPower(-1);
                leftRear.setPower(-1);
                rightFront.setPower(-1);
                rightRear.setPower(-1);
                sleep(2600);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(100);
            }
            else{

                    leftFront.setPower(.15);
                    leftRear.setPower(.15);
                    rightFront.setPower(-.15);
                    rightRear.setPower(-.15);
                    sleep(450);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(-.15);
                    leftRear.setPower(-.15);
                    rightFront.setPower(-.15);
                    rightRear.setPower(-.15);
                    sleep(950);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(.15);
                    leftRear.setPower(.15);
                    rightFront.setPower(.15);
                    rightRear.setPower(.15);
                    sleep(550);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(-.15);
                    leftRear.setPower(-.15);
                    rightFront.setPower(.15);
                    rightRear.setPower(.15);
                    sleep(600);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(.15);
                    leftRear.setPower(-.15);
                    rightFront.setPower(-.15);
                    rightRear.setPower(.15);
                    sleep(1600);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(.15);
                    leftRear.setPower(.15);
                    rightFront.setPower(-.15);
                    rightRear.setPower(-.15);
                    sleep(300);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(.15);
                    leftRear.setPower(.15);
                    rightFront.setPower(-.15);
                    rightRear.setPower(-.15);
                    sleep(650);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(1);
                    leftRear.setPower(-1);
                    rightFront.setPower(-1);
                    rightRear.setPower(1);
                    sleep(1400);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    minKnock.setPower(-1);
                    sleep(1200);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    minKnock.setPower(0);
                    sleep(100);
                    leftFront.setPower(-.15);
                    leftRear.setPower(-.15);
                    rightFront.setPower(.15);
                    rightRear.setPower(.15);
                    sleep(800);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(-.15);
                    leftRear.setPower(.15);
                    rightFront.setPower(.15);
                    rightRear.setPower(-.15);
                    sleep(200);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);
                    leftFront.setPower(-1);
                    leftRear.setPower(-1);
                    rightFront.setPower(-1);
                    rightRear.setPower(-1);
                    sleep(2200);
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                    sleep(100);


            }

        }
        deactivateTfod();


    }
}
