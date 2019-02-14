package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ZerosOneAutoGold", group = "Autonomous")
public class ZerosOneAutoGold extends Nav_Routines {

    boolean goldfound = false;
   // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
       // initVuforiaImage();
        //winchhookdown();
        //go_sideways(270, 0, .4, 10); // Step1
        goRight();
        sleep(200);
        //goRight();
        go_sideways(180, 0, .5, 7); // step2
        sleep(200);
        //go_sideways(0, -25, .5, 2);

        go_sideways(270, 0, 1, 9); // step3
        sleep(200);
        // First Step Rev Expansion need to be updated as 0 , 180 - left ,
        // Side way Logic
        // For the cos and sin calculations below in the mecanum power calcs,
        // angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        goldfound = checktfod();
           if (!goldfound) {

                go_sideways(270, 0, 1, 20); // This is working towward the right
               sleep(200);
               goldfound = checktfod();
               if (!goldfound) {
                   go_sideways(270, 0, 1, 20); // 90
                   sleep(200);
                   goldfound = checktfod();
                  if (!goldfound) {
                          leftcenterright = 1;
                      sleep(200);
                    }
                  else{
                      // Detemine the Gold Mineral in the center.
                      leftcenterright = 3;
                      sleep(200);
                  }
               }
               else
               {
                   // Detemine the Gold Mineral in the center.

                   leftcenterright = 3;
                   sleep(200);
              }
           }
           else
           {
               // Detemine the Gold Mineral in the center.
               leftcenterright = 2;
               sleep(100);
           }
           telemetry.addData("left center right",leftcenterright);

       hitMineral();
        sleep(200);



       // go_sideways(180, 0, 1, 20); // forward
       // hitMineral();
      //  go_forward(10,0,-1,false);
     // go_sideways(90, 0, 0.3,5) ;// knock the gold mineral
       // sleep(500);
      //  go_sideways(270, 0, 3,4) ;
        deactivateTfod();
     //   go_forward(14 + (12 * leftcenterright), 0, .35, false);

    /*  go_forward(14 + (12 * leftcenterright), 0, .35, false);

        turn_to_heading(135);
        go_sideways_to_wall(135, .5, 5, true);
        // go to depot
        wallfollow(28, 135, .4, 5, true, false );
        distancetraveledtodepot = wallfollow(14, 135, .4, 5, true, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 135, -.4, 5, true, false);
        go_forward(14, 135, -.4, true);*/


    }
}
