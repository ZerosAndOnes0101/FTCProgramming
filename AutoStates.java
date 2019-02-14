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

        leftFront.setPower(-.3);
        leftRear.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(-.3);

        sleep(350);

        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);

        sleep(900);

        leftFront.setPower(-.3);
        leftRear.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(-.3);

        sleep(800);
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        sleep(350);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        goldfound = checktfod();
        if (goldfound) {
            leftFront.setPower(-.3);
            leftRear.setPower(-.3);
            rightFront.setPower(-.3);
            rightRear.setPower(-.3);
            sleep(1500);
            leftFront.setPower(.3);
            leftRear.setPower(.3);
            rightFront.setPower(.3);
            rightRear.setPower(.3);
            sleep(1500);
            leftFront.setPower(.3);
            leftRear.setPower(-.3);
            rightFront.setPower(-.3);
            rightRear.setPower(.3);
            sleep(3200);
            leftFront.setPower(.3);
            leftRear.setPower(.3);
            rightFront.setPower(-.3);
            rightRear.setPower(-.3);
            sleep(350);
            leftFront.setPower(-1);
            leftRear.setPower(-1);
            rightFront.setPower(-1);
            rightRear.setPower(-1);
            sleep(1400);

        }
        else {

            leftFront.setPower(.3);
            leftRear.setPower(-.3);
            rightFront.setPower(-.3);
            rightRear.setPower(.3);
           sleep(600);

           goldfound = checktfod();

            if (goldfound) {

                leftFront.setPower(-.3);
                leftRear.setPower(-.3);
                rightFront.setPower(-.3);
                rightRear.setPower(-.3);
                sleep(1500);
                leftFront.setPower(.3);
                leftRear.setPower(.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                sleep(1500);
                leftFront.setPower(.3);
                leftRear.setPower(-.3);
                rightFront.setPower(-.3);
                rightRear.setPower(.3);
                sleep(2500);
                leftFront.setPower(.3);
                leftRear.setPower(.3);
                rightFront.setPower(-.3);
                rightRear.setPower(-.3);
                sleep(350);
                leftFront.setPower(-1);
                leftRear.setPower(-1);
                rightFront.setPower(-1);
                rightRear.setPower(-1);
                sleep(1400);


            }
    else {
                leftFront.setPower(.3);
                leftRear.setPower(-.3);
                rightFront.setPower(-.3);
                rightRear.setPower(.3);
                sleep(600);

                goldfound = checktfod();

                if (goldfound) {

                    leftFront.setPower(-.3);
                    leftRear.setPower(-.3);
                    rightFront.setPower(-.3);
                    rightRear.setPower(-.3);
                    sleep(1500);
                    leftFront.setPower(.3);
                    leftRear.setPower(.3);
                    rightFront.setPower(.3);
                    rightRear.setPower(.3);
                    sleep(1500);
                    leftFront.setPower(.3);
                    leftRear.setPower(-.3);
                    rightFront.setPower(-.3);
                    rightRear.setPower(.3);
                    sleep(1000);
                    leftFront.setPower(.3);
                    leftRear.setPower(.3);
                    rightFront.setPower(-.3);
                    rightRear.setPower(-.3);
                    sleep(350);
                    leftFront.setPower(-1);
                    leftRear.setPower(-1);
                    rightFront.setPower(-1);
                    rightRear.setPower(-1);
                    sleep(1400);


                }

            }

        }



        deactivateTfod();



    }
}
