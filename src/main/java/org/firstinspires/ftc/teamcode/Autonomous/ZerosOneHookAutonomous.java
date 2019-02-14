package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ZerosOneHookAutonomous", group = "Autonomous")
public class ZerosOneHookAutonomous extends Nav_Routines {

    boolean goldfound = false;
   // int leftrightcenter = 1; // 1 = left,  2 = right, 3= center
    double distancetraveledtodepot = 0;
    int leftcenterright = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
       // initVuforiaImage();
        //zno winchup();
        winchhookdown();

        // First Step Rev Expansion need to be updated as 0 , 180 - left ,
        // Side way Logic
        // For the cos and sin calculations below in the mecanum power calcs,
        // angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.


        go_sideways(180, 0, .5, 2);
        go_forward(2,0,.4,false);
        goldfound = checktfod();
           if (!goldfound) {
               //go_forward(14, 0, -.4, false);
               //jv go_sideways(270, 0, .3, 5); // Confirmed for right side
               go_sideways(180, 0, 1, 20);
               goldfound = checktfod();
               if (!goldfound) {
                  go_sideways(0, 0, 1, 20);
                  goldfound = checktfod();
                  if (!goldfound) {
                      go_sideways(0, 0, 1, 20);
                      goldfound = checktfod();
                      if (!goldfound) {
                          // testing
                      }
                      else{
                          // Detemine the Gold Mineral in the center.
                          leftcenterright = 1;
                      }
                  }
                  else{
                      // Detemine the Gold Mineral in the center.
                      leftcenterright = 2;
                  }
               }
               else
               {
                   // Detemine the Gold Mineral in the center.
                   leftcenterright = 3;
               }
           }
           else
           {
               // Detemine the Gold Mineral in the center.
               leftcenterright = 2;
           }
        //hitMineral();
      go_sideways(90, 0, 0.3,5) ;// knock the gold mineral
        sleep(500);
        go_sideways(270, 0, 3,4) ;
        deactivateTfod();

      go_forward(14 + (12 * leftcenterright), 0, .35, false);

        turn_to_heading(135);
        go_sideways_to_wall(135, .5, 5, true);
        // go to depot
        wallfollow(28, 135, .4, 5, true, false );
        distancetraveledtodepot = wallfollow(14, 135, .4, 5, true, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 135, -.4, 5, true, false);
        go_forward(14, 135, -.4, true);


    }
}
