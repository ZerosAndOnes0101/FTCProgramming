package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoStatesDepot", group = "Autonomous")
public class AutoStatesDepot extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();

        winchhookdown();

        strafeRight(.15, 350, 100);

        forward(.15, 650, 100);

        strafeRight(.15, 800, 100);


        goldfound = checktfodSampling();
        if (goldfound) {
            strafeRight(.15,500,100);

            forward(.15, 1100, 100);

            turnRight(.15,1150,100);

            strafeLeft(.15, 500, 100);

            backward(.15,950,100);

            deposit(1200, 100);

            turnRight(.15,1150,100);

            strafeRight(.15,1150,100);

            forward(1, 3000, 100);


        } else {


            strafeLeft(.15, 960, 100);

            turnRight(.15, 150, 100);


            goldfound = checktfodSampling();
            if (goldfound) {

                forward(.15, 1200, 100);

                turnRight(.15, 1150, 100);

                strafeLeft(.15,700,100);

                backward(.15,200,100);

                deposit(1200, 100);

                strafeRight(.15,400,100);

               turnRight(.15,1000,100);

               strafeRight(.15,700,100);

                forward(1, 3000, 100);


            } else {

                turnLeft(.15, 650, 100);

                forward(.15, 1000, 100);

                turnLeft(.15, 900, 100);

                backward(.15,1200,100);

                deposit(1200, 100);

                strafeRight(.15,500,100);

                forward(1, 2500, 100);


            }

        }
        deactivateTfod();


    }


}