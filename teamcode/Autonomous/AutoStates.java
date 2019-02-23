package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoStates", group = "Autonomous")
public class AutoStates extends Nav_Routines {

    boolean goldfound = false;
    // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;
    boolean isObjExist = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();

        winchhookdown();
        strafeRight(.15,350,100);

        forward(.15,550,100);

        turnLeft(.15,200,100);

        strafeRight(.15,1000,100);
		


           /* isObjExist = checkObject();
            if (isObjExist){
                telemetry.addData("# Objects Detected Status", isObjExist);
                telemetry.update();
            }
            else
            {
                strafeLeft(.15,100,100);
            }*/

        goldfound = checktfodSampling();
        if (goldfound) {
            forward(.15,700,100);

           backward(.15,550,100);

            strafeLeft(.5,2400,100);

            turnRight(.15,600,100);

            strafeLeft(.15,1000 ,100);

            backward(.3,900,100);

            deposit(1200,100);

            forward(1,2000,100);

        } else {

            strafeLeft(.15,1000,100);

            turnRight(.15,300,100);


           /* isObjExist = false;

                isObjExist = checkObject();
                if (isObjExist){
                    telemetry.addData("# Objects Detected Status", isObjExist);
                    telemetry.update();
                }
                else
                {
                    telemetry.addData("# Moved to Left", isObjExist);
                    telemetry.update();
                    strafeLeft(.15,10,100);
                }*/

            goldfound = checktfodSampling();
            if (goldfound) {
                forward(.15,750,100);

                backward(.15,600,100);

                strafeLeft(.5,1300,100);

                turnRight(.3,400,100);

                strafeLeft(.15,800,100);

                backward(.15,1400,100);

                deposit(1200,100);

                strafeLeft(.15,300,100);

                forward(.5,2600,100);

            }
            else{
                    turnLeft(.15,450,100);

                    forward(.15,800,100);

                    backward(.15,450,100);

                    strafeLeft(.15,2000,100);

                    turnRight(.15,750,100);

                    strafeLeft(.15,700,100);

                    backward(.15,1100,100);

                    deposit(1200,100);

                    forward(1,3000,100);

            }

        }
        deactivateTfod();


    }
}