package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoStates", group = "Autonomous")
public class AutoStates extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;
    boolean checkObj = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();

        winchhookdown();
        strafeRight(.15,350,100);


        forward(.15,550,100);

        strafeRight(.15,900,100);

        checkObj = checkObject();
        if (checkObj) {

        }

        if (goldfound) {
            forward(.15,700,100);

           backward(.15,450,100);

            strafeLeft(.5,2800,100);

            turnLeft(.15,800,100);

            forward(.3,400,100);

            strafeLeft(.3,1300,100);

            deposit(1200,100);

            turnRight(.15,1000,100);

            strafeLeft(.3,1000,100);

            forward(.5,2600,100);

        } else {

            strafeLeft(.15,1100,100);

            turnRight(.15,300,100);

            goldfound = checktfod();
            if (goldfound) {
                forward(.15,750,100);

                backward(.15,600,100);

                strafeLeft(.5,1800,100);

                turnLeft(.3,800,100);

                forward(.15,400,100);

                strafeLeft(.3,1200,100);

                deposit(1200,100);

                turnRight(.15,800,100);

                strafeLeft(.15,1000,100);

                forward(.5,2600,100);

            }
            else{
                    turnLeft(.15,450,100);

                    forward(.15,950,100);

                    backward(.15,550,100);

                    turnRight(.15,600,100);

                    strafeLeft(.15,1600,100);

                    turnLeft(.3,600,100);

                    forward(.15,400,100);

                    strafeLeft(.5,1800,100);

                    deposit(1200,100);

                    turnRight(.15,800,100);

                    strafeLeft(.15,1000,100);

                    forward(.7,2200,100);
                    
            }

        }
        deactivateTfod();


    }
}
