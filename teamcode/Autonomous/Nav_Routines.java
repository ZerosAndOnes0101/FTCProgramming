package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.DbgLog;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

abstract public class Nav_Routines extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor armJoint = null;
    Servo boxJoint;
    CRServo minKnock;

    DcMotor winchmotor;

    DcMotor mil1;
    CRServo mil2;

    Servo mineralknockservo;

    double batteryPower;
    double newPower;
    //CRServo mineralslidesblockservo;

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    public static final double CAMERA_FROM_FRONT = 244.0; // Millimeters. ** Convert to Meet 3 build

    private OpenGLMatrix lastLocation = null;

    Rev2mDistanceSensor leftdistancesensor;
    Rev2mDistanceSensor frontdistancesensor;
    Rev2mDistanceSensor rightdistancesensor;


    BNO055IMU imu;
    Orientation angles;

    double gs_previous_speed;
    double gs_previous_ticks_traveled;
    boolean gs_first_run = true;
    ElapsedTime gs_speed_timer = new ElapsedTime();

    private double wheel_encoder_ticks = 537.6;
    private double wheel_diameter = 3.75;  // size of wheels
    public double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

    public double goforwardstopdetect = 2;
    public int ignorecrater = 240;

    int mil1startticks;
    int mil2startticks;
    int winchstartticks;

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    TFObjectDetector tfod;


    public double knownX;
    public double knownY;
    public double knownHeading;
    public boolean seeking = false;
    public List<VuforiaTrackable> allTrackables = null;
    double current_heading;

    double volts;


    public void Nav_Init() {
        leftFront = hardwareMap.dcMotor.get("FL");
        rightFront = hardwareMap.dcMotor.get("FR");
        leftRear = hardwareMap.dcMotor.get("BL");
        rightRear = hardwareMap.dcMotor.get("BR");
        boxJoint = hardwareMap.get(Servo.class, "boxJoint");
        armJoint = hardwareMap.get(DcMotor.class, "lift");
        winchmotor = hardwareMap.dcMotor.get("hook"); //claw
        minKnock = hardwareMap.get(CRServo.class, "min");

        //mil1 = hardwareMap.dcMotor.get("lift"); //claw
        //mil2 = hardwareMap.crservo.get("intake");

        mineralknockservo = hardwareMap.servo.get("mks");
        //mineralslidesblockservo = hardwareMap.crservo.get("boxJoint");
        /*frontdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "fds");
        leftdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "lds");
        rightdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "rds");*/


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        winchmotor.setDirection(DcMotor.Direction.REVERSE);

        mil1startticks = 0;
        mil2startticks = 0;
        winchstartticks = winchmotor.getCurrentPosition();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        winchmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*leftdistancesensor.initialize();
        frontdistancesensor.initialize();
        rightdistancesensor.initialize();*/


        //  mineralslidesblockservo.setPower(.5);
        //  mineralslidesblockservo.setPosition(0.5)

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        parameters.vuforiaLicenseKey = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.92;
            //  tfodParameters.useObjectTracker = false;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

            tfod.loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Initialize!! ", "Start");
        waitForStart();


    }


    public void hitMineral() {
        leftFront.setPower(-1);
        rightRear.setPower(-1);
        rightRear.setPower(-1);
        leftRear.setPower(-1);
        sleep(3000);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

    }

    public void hitMarkerMineral() {
        leftFront.setPower(-0.5);
        rightRear.setPower(-0.5);
        rightRear.setPower(-0.5);
        leftRear.setPower(-0.5);
        sleep(3000);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

    }

    public void initVuforiaImage() {
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();
    }

    public void goToImage() {
        do {
            String field = "I see";
            String imageName = "nothing";
            String report = "";

            //  Find an image, and get robot's location relative to it.
            imageName = findImage(allTrackables);

            if (!imageName.equals("nothing")) {
                //  Go toward detected beacon image.
                report = stepTowardsImage(imageName);
                if (knownY < CAMERA_FROM_FRONT) {
                    report += "Arrived. Stopping.";
                    stopDriveMotors();
                    telemetry.addData(field, report);
                    telemetry.update();
                    break;
                }
            } else {
                report = "nothing. Stopping.";
                // Stop
                // ** Maybe turn robot toward last image bearing, hoping to pick it up again
                stopDriveMotors();
            }

            telemetry.addData(field, report);
            telemetry.update();
        } while (true);
    }

    public void stopDriveMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    //   ** CONVERTED, but sloppy due to wheel slip.
    private String stepTowardsImage(String whichImage) {
        String report = "";
        //double leftFrontSpeed = 0.0;
        //double rightFrontSpeed = 0.0;
        //double leftRearSpeed = 0.0;
        //double rightRearSpeed = 0.0;
        setXYHeading(lastLocation);
        double remainingX = knownX;
        double remainingY = knownY - CAMERA_FROM_FRONT;  // Millimeters away from stopping.

        // Angles in radians
        // knownHeading is zero if camera directly facing plane of the image, independent
        // of camera position.
        // Positive if pointing to left of normal to that plane through the camera.
        //Orientation orientation = Orientation.getOrientation(lastLocation,
        //        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        //knownHeading = orientation.thirdAngle;

        // Image bearing is zero if camera anywhere on normal to image, independent of camera
        //   orientation. Positive if image is left of normal to plane of image, passing through
        //   the camera.
        double imageBearing = -Math.atan2(knownX, knownY);
        //imageBearing *= 180 / Math.PI;
        // Guidance tuning constants
        final double APPROACH_SPEED = 0.20; // ** .1 is too weak.
        //  ** May become a tunable variable, slower for smaller abs(X).
        final double FORE_AFTER = 2.5; // Controls strength of X correction
        //  ** Also may become a tunable variable, gentler for smaller abs(X).
        final double HEADING_CORRECTOR = 0.35; // Controls strength of X correction
        double targetHeading = imageBearing * HEADING_CORRECTOR;
        report += whichImage + String.format(" at X, Y: %6.0fmm", knownX) + String.format(", %6.0fmm", knownY);
        report += String.format("\nImage bearing %6.2f radians", imageBearing);

        report += String.format("\nRobot heading %6.2f radians", knownHeading);

        /* Three components must be controlled.
             Y error: distance along normal to beacon image. Distance robot must still
               travel toward that image.
             X error: side to side error. Distance from that normal to the beacon image,
               approximately distance from center of white line.
             Heading error. Angle of robot's Y axis with respect to that normal to the
               beacon image.
           */
        //  Major component will be Y error. The others are perturbations from that.
        //    Slide to port. ** Looks good.

        double leftFrontSpeed = -APPROACH_SPEED;
        double rightFrontSpeed = APPROACH_SPEED;
        double leftRearSpeed = APPROACH_SPEED;
        double rightRearSpeed = -APPROACH_SPEED;

        //  X error correction. Modifies wheel commands that reduce Y error to produce
        //    angled motion towards beacon, reducing sideways error. Strength of correction
        //    decays as X error decays. X error is measured by imageBearing.
        // ** Looks good
        leftFrontSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        leftRearSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        rightFrontSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;
        rightRearSpeed -= FORE_AFTER * imageBearing * APPROACH_SPEED;

        //  Heading adjustment component. Modifies combined Y and X error corrections
        //    to turn robot. This correction also decays, governed by knownHeading.
        //    ** Looks good.
        leftFrontSpeed += HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        leftRearSpeed += HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        rightFrontSpeed -= HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;
        rightRearSpeed -= HEADING_CORRECTOR * knownHeading * APPROACH_SPEED;

        //  Apply all 3 corrections to robot wheels.
        leftFront.setPower(leftFrontSpeed);
        leftRear.setPower(leftRearSpeed);
        rightFront.setPower(rightFrontSpeed);
        rightRear.setPower(rightRearSpeed);

        report += String.format("\nWheel speeds: lf %6.2f  rf %6.2f", leftFrontSpeed, rightFrontSpeed);
        report += String.format(" lr %6.2f  rr %6.2f", leftRearSpeed, rightRearSpeed);
        if (remainingY < 0) {
            // Just report, and let driver contemplate the situation.
            report += " I'm too close. Stopping.";
            // stopDriveMotors();
        }
        RobotLog.a(report);
        return report;
    }

    //  Set robot heading and coordinates from some location object.
    //  ** CONVERTED.
    public void setXYHeading(OpenGLMatrix aLocation) {
        VectorF translation = aLocation.getTranslation();
        knownX = -translation.get(0);
        knownY = -translation.get(1);
        Orientation orientation = Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        knownHeading = orientation.thirdAngle;
    }

    //   CONVERTED.
    public String findImage(List<VuforiaTrackable> allTrackables) {
        String imageName = "nothing";
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                imageName = trackable.getName();
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    break; // Never interested in more than one image.
                }
            }
        }
        return imageName;
    }

    public void turn_to_heading(double target_heading) {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prevheading = 0;
        ElapsedTime timeouttimer = new ElapsedTime();

        DbgLog.msg("10435 Starting TURN_TO_HEADING");
        current_heading = currentheadingreading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeouttimer.reset();
        prevheading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeouttimer.seconds() < 2) {

            wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;

            if (go_right) {
                wheel_power = -wheel_power;
            }

            rightFront.setPower(wheel_power);
            rightRear.setPower(wheel_power);
            leftFront.setPower(-wheel_power);
            leftRear.setPower(-wheel_power);

            current_heading = currentheadingreading();

            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prevheading) > 1) {
                timeouttimer.reset();
                prevheading = current_heading;
            }

            DbgLog.msg("10435 TURN_TO_HEADING" + " go right: " + go_right + " degrees to turn: " + degrees_to_turn + " wheel power: " + wheel_power + " current heading: " + current_heading + "Wheel power: " + Double.toString(wheel_power));
        }

        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        DbgLog.msg("10435 ending TURN_TO_HEADING" + Double.toString(target_heading) + "  Final heading:" + Double.toString(current_heading) + "  After set power 0:" + Double.toString(angles.firstAngle));

    } // end of turn_to_heading


    public void go_forward(double inches_to_travel, double heading, double speed, boolean gotocrater) {

        DbgLog.msg("10435 Starting GO_FORWARD inches:" + Double.toString(inches_to_travel) + " heading:" + Double.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;
        boolean craterhit = false;

        ElapsedTime timeouttimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && timeouttimer.seconds() < goforwardstopdetect && !craterhit) {

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            if (gotocrater) {
                if (highest_ticks_traveled - lowest_ticks_traveled > 75) {
                    craterhit = true;
                }
            }

            actual_speed = getSpeed(lowest_ticks_traveled);

            if (actual_speed > 0.1) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 GO_FORWARD ticks_traveled: L:" + Double.toString(lowest_ticks_traveled_l) + " R:" + Double.toString(lowest_ticks_traveled_r) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
            }

            destination_reached = (lowest_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 GO_FORWARD slowing down: remaining_inches:" + Double.toString(remaining_inches) + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending GO_FORWARD: opModeIsActive:" + Boolean.toString(opModeIsActive()) + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch)) + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch)) + " destination_reached:" + Boolean.toString(destination_reached) + " timouttimer:" + Double.toString(timeouttimer.seconds()) + " lowest ticks traveled:" + Integer.toString(lowest_ticks_traveled) + " highest ticks traveled:" + Integer.toString(highest_ticks_traveled));

    } // end of go_forward

    public double go_sideways(double angledegrees, double heading, double power, double inches) {

        DbgLog.msg("10435 Starting go_sideways" + " angledegrees:" + Double.toString(angledegrees) + " heading:" + Double.toString(heading) + " power:" + Double.toString(power) + " inches:" + Double.toString(inches));

        double stickpower = power;
        double angleradians;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower = 0;
        boolean mineralclose = false;
        boolean destinationreached = false;
        final double ticksperinch = 47;
        int ticks_to_travel;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        ticks_to_travel = (int) (inches * ticksperinch);

        // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
        // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
        // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

        // This converts from *our* degrees to radians used by the mecanum power calcs.
        // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
        if (angledegrees < 270) {
            angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
        } else {
            angleradians = (450 - angledegrees) * Math.PI / 180;
        }

        angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

        while (opModeIsActive() && !mineralclose && !destinationreached) {

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);


            if (highest_ticks_traveled >= ticks_to_travel) {
                destinationreached = true;
            }

            turningpower = -go_straight_adjustment(heading) * (power * 2);

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower;


            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);

            DbgLog.msg("10435 go_sideways" + " turningpower:" + Double.toString(turningpower) + " leftfrontpower" + Double.toString(leftfrontpower) + " rightfrontpower" + Double.toString(rightfrontpower) + " leftrearpower" + Double.toString(leftrearpower) + " rightrearpower" + Double.toString(rightrearpower));
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(50);

        return highest_ticks_traveled / ticksperinch;
    }

    public void go_sideways_to_wall(double heading, double power, double walldistance, boolean useleft) {
        double stickpower = power;
        double angleradians = 90;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double turningpower;
        double inchesreadfromwall;
        double angledegrees;

        if (useleft) {
            // inchesreadfromwall = leftdistancesensor.getDistance(DistanceUnit.INCH);
            inchesreadfromwall = 5;
        } else {
            // inchesreadfromwall = rightdistancesensor.getDistance(DistanceUnit.INCH);
            inchesreadfromwall = 5;
        }

        if (useleft && walldistance - inchesreadfromwall > 0 || !useleft && walldistance - inchesreadfromwall < 0) {
            angledegrees = 90;
        } else {
            angledegrees = 270;
        }

        DbgLog.msg("10435 Starting go_sideways_to_wall" + " angledegrees:" + Double.toString(angledegrees) + " heading:" + Double.toString(heading) + " power:" + Double.toString(power) + " walldistance:" + Double.toString(walldistance) + " useleft:" + Boolean.toString(useleft));

        while (opModeIsActive() && Math.abs(inchesreadfromwall - walldistance) > 1) {

            if (useleft) {
                // zno  inchesreadfromwall = leftdistancesensor.getDistance(DistanceUnit.INCH);
                inchesreadfromwall = 5;
            } else {
                // zno  inchesreadfromwall = rightdistancesensor.getDistance(DistanceUnit.INCH);
                inchesreadfromwall = 5;
            }

            if (useleft && walldistance - inchesreadfromwall > 0 || !useleft && walldistance - inchesreadfromwall < 0) {
                angledegrees = 90;
            } else {
                angledegrees = 270;
            }

            // For the cos and sin calculations below in the mecanum power calcs, angleradians = 0 is straight to the right and 180 is straight to the left.
            // Negative numbers up to -180 are backward.  Postive numbers up to 180 are forward.
            // We subtract 90 from it then convert degrees to radians because *our* robot code thinks of 0 degrees as forward, 90 as right, 180 as backward, 270 as left.

            // This converts from *our* degrees to radians used by the mecanum power calcs.
            // Upper left quadrant (degrees > 270) is special because in that quadrant as our degrees goes up, radians goes down.
            if (angledegrees == 90) {
                angleradians = ((angledegrees - 90) * -1) * Math.PI / 180;
            } else if (angledegrees == 270) {
                angleradians = (450 - angledegrees) * Math.PI / 180;
            }

            angleradians = angleradians - Math.PI / 4; //adjust by 45 degrees for the mecanum wheel calculations below

            turningpower = -go_straight_adjustment(heading) * (power * 2);

            leftfrontpower = stickpower * Math.cos(angleradians) + turningpower;
            rightfrontpower = stickpower * Math.sin(angleradians) - turningpower;
            leftrearpower = stickpower * Math.sin(angleradians) + turningpower;
            rightrearpower = stickpower * Math.cos(angleradians) - turningpower;

            leftFront.setPower(leftfrontpower);
            rightFront.setPower(rightfrontpower);
            leftRear.setPower(leftrearpower);
            rightRear.setPower(rightrearpower);


            DbgLog.msg("10435 go_sideways_to_wall" + " inchesreadfromwall: " + Double.toString(inchesreadfromwall) + " turningpower:" + Double.toString(turningpower) + " leftfrontpower" + Double.toString(leftfrontpower) + " rightfrontpower" + Double.toString(rightfrontpower) + " leftrearpower" + Double.toString(leftrearpower) + " rightrearpower" + Double.toString(rightrearpower));

        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(50);
    }

    public double wallfollow(double inches_to_travel, double heading, double speed, double walldistance, boolean left, boolean finddepot) {

        DbgLog.msg("10435 Starting WALLFOLLOW inches:" + Double.toString(inches_to_travel) + " heading:" + Double.toString(heading) + " speed:" + Double.toString(speed));

        ElapsedTime log_timer = new ElapsedTime();

        double current_speed = .05;
        int ticks_to_travel;
        boolean destination_reached = false;
        boolean going_backwards = false;
        double speed_increase = .05;
        double actual_speed;
        double lagreduction = 2.125;
        int start_position_l_Front;
        int start_position_l_Rear;
        int start_position_r_Front;
        int start_position_r_Rear;
        int previous_ticks_traveled_L = 0;
        int ticks_traveled_l_Front;
        int ticks_traveled_l_Rear;
        int ticks_traveled_r_Front;
        int ticks_traveled_r_Rear;
        int lowest_ticks_traveled_l = 0;
        int lowest_ticks_traveled_r = 0;
        int lowest_ticks_traveled = 0;
        int highest_ticks_traveled_l;
        int highest_ticks_traveled_r;
        int highest_ticks_traveled = 0;
        int average_ticks_traveled = 0;
        double remaining_inches;
        double previous_log_timer = 0;
        double power_adjustment;
        double distance_off;
        boolean wallfound = false;

        ElapsedTime timeouttimer = new ElapsedTime();

        if (speed < 0) {
            inches_to_travel = inches_to_travel * 1.08;
            speed_increase = -speed_increase;
            current_speed = -current_speed;
            going_backwards = true;
        }

        inches_to_travel = inches_to_travel - lagreduction;

        ticks_to_travel = (int) (inches_to_travel * ticks_per_inch);

        start_position_l_Front = leftFront.getCurrentPosition();
        start_position_l_Rear = leftRear.getCurrentPosition();
        start_position_r_Front = rightFront.getCurrentPosition();
        start_position_r_Rear = rightRear.getCurrentPosition();

        log_timer.reset();
        timeouttimer.reset();

        gs_first_run = true;

        while (opModeIsActive() && !destination_reached && timeouttimer.seconds() < goforwardstopdetect && !wallfound) {

            if (finddepot) {
                wallfound = checkfrontdistancesensor();
            }

            if (left) {
                // zno  distance_off = leftdistancesensor.getDistance(DistanceUnit.INCH) - walldistance;
                distance_off = 1;
            } else {
                // zno  distance_off = rightdistancesensor.getDistance(DistanceUnit.INCH) - walldistance;
                distance_off = 1;
            }

            if (Math.abs(distance_off) >= 1.5) {
                go_sideways_to_wall(heading, .25, walldistance, left);
            }

            current_speed = current_speed + speed_increase;  // this is to slowly ramp up the speed so we don't slip
            if (Math.abs(current_speed) > Math.abs(speed)) {
                current_speed = speed;
            }

            power_adjustment = go_straight_adjustment(heading);

            rightFront.setPower(current_speed + power_adjustment);
            rightRear.setPower(current_speed + power_adjustment);
            leftFront.setPower(current_speed - power_adjustment);
            leftRear.setPower(current_speed - power_adjustment);

            ticks_traveled_l_Front = Math.abs(leftFront.getCurrentPosition() - start_position_l_Front);
            ticks_traveled_l_Rear = Math.abs(leftRear.getCurrentPosition() - start_position_l_Rear);
            ticks_traveled_r_Front = Math.abs(rightFront.getCurrentPosition() - start_position_r_Front);
            ticks_traveled_r_Rear = Math.abs(rightRear.getCurrentPosition() - start_position_r_Rear);

            // of the 4 wheels, determines lowest ticks traveled
            lowest_ticks_traveled_l = Math.min(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            lowest_ticks_traveled_r = Math.min(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            lowest_ticks_traveled = Math.min(lowest_ticks_traveled_l, lowest_ticks_traveled_r);

            // of the 4 wheels, determines highest ticks traveled
            highest_ticks_traveled_l = Math.max(ticks_traveled_l_Front, ticks_traveled_l_Rear);
            highest_ticks_traveled_r = Math.max(ticks_traveled_r_Front, ticks_traveled_r_Rear);
            highest_ticks_traveled = Math.max(highest_ticks_traveled_l, highest_ticks_traveled_r);

            average_ticks_traveled = (ticks_traveled_l_Front + ticks_traveled_l_Rear + ticks_traveled_r_Front + ticks_traveled_r_Rear) / 4;

            actual_speed = getSpeed(average_ticks_traveled);

            if (actual_speed > 0.1) {  // if we're going less than this we aren't moving.
                timeouttimer.reset();
            }

            if (lowest_ticks_traveled_l != previous_ticks_traveled_L && log_timer.seconds() - previous_log_timer > .1) {
                DbgLog.msg("10435 WALLFOLLOW ticks_traveled: L:" + Double.toString(lowest_ticks_traveled_l) + " R:" + Double.toString(lowest_ticks_traveled_r) + " actual_speed:" + actual_speed + " current speed:" + current_speed + " speed:" + speed);
                previous_log_timer = log_timer.seconds();
                previous_ticks_traveled_L = lowest_ticks_traveled_l;
            }

            destination_reached = (average_ticks_traveled >= ticks_to_travel);

            remaining_inches = inches_to_travel - ((double) lowest_ticks_traveled / ticks_per_inch);

            if (remaining_inches <= actual_speed && Math.abs(speed) > .2) {
                speed = .2;
                if (going_backwards) {
                    speed = -speed;
                }
                DbgLog.msg("10435 WALLFOLLOW slowing down: remaining_inches:" + Double.toString(remaining_inches) + " lowest_ticks_traveled:" + Integer.toString(lowest_ticks_traveled));
            }

        }
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);


        sleep(100);
        DbgLog.msg("10435 ending WALLFOLLOW: opModeIsActive:" + Boolean.toString(opModeIsActive()) + " distance traveled L:" + Double.toString((lowest_ticks_traveled_l / ticks_per_inch)) + " distance traveled R:" + Double.toString((lowest_ticks_traveled_r / ticks_per_inch)) + " destination_reached:" + Boolean.toString(destination_reached) + " timouttimer:" + Double.toString(timeouttimer.seconds()) + " lowest ticks traveled:" + Integer.toString(lowest_ticks_traveled) + " highest ticks traveled:" + Integer.toString(highest_ticks_traveled));

        return (inches_to_travel - 1) - (average_ticks_traveled / ticks_per_inch);
    }

    public double getBatteryVoltage(double power) {
        double batt_power = 1;

        volts = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (volts >= 12.8 && volts <= 13.2) {
            batt_power = 0.30;
        } else if (volts >= 13.3 && volts <= 13.8) {
            batt_power = .15;
        } else if (volts >= 13.9) {
            batt_power = 0.10;
        }

        return batt_power*power;
    }

    public long getTimeBasedVoltage(long sleeptime) {
        double new_temp;
        volts = hardwareMap.voltageSensor.iterator().next().getVoltage();
        long new_sleep_time=1;
        long temp_sleep=1;
        new_temp=(1-((volts-13)*0.1));
        if (new_temp > 0) {
            temp_sleep= (long)new_temp ;
            new_sleep_time= sleeptime * temp_sleep;
        }
        else
        {
            new_sleep_time=sleeptime;
        }
        return new_sleep_time;
    }

    public void goRight() {
        go_sideways(270, 0, .4, 10); // sideway left
        sleep(100);

        current_heading = currentheadingreading();
        telemetry.addData("goRight", "Running at %2f", current_heading);
        telemetry.update();

    }

  /*  public void strafeRight(double power){
        double newPower= power*.4;
        go_sideways(270, 0, newPower, 10); // sideway left
        sleep(100);
    }*/

    public void strafeRight(double power, int sleeptime, int endSleeptime) {
        /* batteryPower=getBatteryVoltage();
         newPower=batteryPower*power;
         int newTime= sleeptime+ (int)Math.abs(1-batteryPower)* 100;*/
        leftFront.setPower(-power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void forward(double power, int sleeptime, int endSleeptime) {
        // batteryPower=getBatteryVoltage();
        //newPower=batteryPower*power;
        //int newTime= sleeptime+ (int)Math.abs(1-batteryPower)* 100;
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void backward(double power, int sleeptime, int endSleeptime) {
        //  int newTime= sleeptime+ (int)Math.abs(1-batteryPower)* 100;
        //batteryPower=getBatteryVoltage();
        //newPower=batteryPower*power;
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void strafeLeft(double power, int sleeptime, int endSleeptime) {
        //  batteryPower = getBatteryVoltage();
        //newPower = batteryPower * power;
        leftFront.setPower(power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnRight(double power, int sleeptime, int endSleeptime) {
        // batteryPower = getBatteryVoltage();
        //int newTime= sleeptime+ (int)Math.abs(1-batteryPower)* 100;
        //newPower = batteryPower * power;
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnLeft(double power, int sleeptime, int endSleeptime) {
        //    batteryPower = getBatteryVoltage();
        //  int newTime= sleeptime+ (int)Math.abs(1-batteryPower)* 100;
        //newPower = batteryPower * power;
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void deposit(int sleeptime, int endSleeptime) {

        minKnock.setPower(-1);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        minKnock.setPower(0);
        sleep(endSleeptime);

    }


    public void winchhookdown() {
        int winchticks;
        winchmotor.setPower(-1);
        sleep(6500);
        winchmotor.setPower(0);
        //winchmotor.setPower(0);

        current_heading = currentheadingreading();
        telemetry.addData("WinchDown", "Running at %2f", current_heading);
        telemetry.update();
    }

    public void winchhookup() {
        int winchticks;
        winchmotor.setPower(1);
        sleep(6500);
        winchmotor.setPower(0);
        //winchmotor.setPower(0);
    }

    public void winchdown() {
        int winchticks;
        winchmotor.setPower(-1);
        sleep(4700);
        winchmotor.setPower(0);
        //winchticks = winchmotor.getCurrentPosition() - winchstartticks;
        // winchticks = 300;
        //while (winchticks > 200) {
        //    winchticks = winchmotor.getCurrentPosition() - winchstartticks;
        //   winchmotor.setPower(-1);
    }
    //winchmotor.setPower(0);


    // public void winchup() {
    //   boolean magnetistouching = false;
    // winchmotor.setPower(-1);
    //while (!magnetistouching && opModeIsActive()) {
    //zno magnetistouching = !magneticlimitswitch.getState();
    //  magnetistouching = false;
    //if (!magnetistouching) {
    //  winchmotor.setPower(1);
    //} else {
    //    winchmotor.setPower(0);
    //}
    //}
    //}

    public void deploymarker() {
        int mil1ticks;
        boolean liftisout = false;

        while (!liftisout && opModeIsActive()) {
            //zno  mil1ticks = mil1startticks - mil1.getCurrentPosition();
            mil1ticks = 180;
            if (mil1ticks < 180) {
                mil1.setPower(-.4);
                mil2.setPower(-.4);
            } else {
                mil1.setPower(0);
                mil2.setPower(0);
                liftisout = true;
            }
            telemetry.addData("Mil1Ticks", mil1ticks);
            telemetry.update();
        }

        boolean liftisback = false;

        while (!liftisback && opModeIsActive()) {
            mil1ticks = 30;
            // zno mil1ticks = mil1startticks - mil1.getCurrentPosition();
            if (mil1ticks > 30) {
                mil1.setPower(.4);
                mil2.setPower(.4);
            } else {
                mil1.setPower(0);
                mil2.setPower(0);
                liftisback = true;
            }
            telemetry.addData("Mil1Ticks", mil1ticks);
            telemetry.update();
        }

    }

    public void deploymarker2() {
        //mineralslidesblockservo.setPower(.15);
        sleep(1000);
        //mineralslidesblockservo.setPower(.5);
    }

    public boolean checkObject() {
        boolean found = false;
        int loopcount = 0;
        boolean objNotFound= false;

        while (opModeIsActive() && loopcount < 2 && !objNotFound) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        found=true;
                        break;
                    }
                    else{
                        objNotFound= true;
                    }
                }
                loopcount = loopcount + 1;
            }
            telemetry.addData("loopcount", loopcount);
            sleep(100);
        }
      return found;
    }


    public boolean checktfodSampling() {
        boolean found;
        int loopcount = 0;
        int noobjcount = 0;

        telemetry.update();  // clear out telemetry?
        int closestmineral = 0;
        double closestbottom = 0;
        String closestlabel = "";

        while (opModeIsActive() && loopcount < 5 && noobjcount < 2) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {

                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        int mineralcounter;
                        mineralcounter = 0;
                        closestmineral = 0;
                        closestbottom = 0;
                        loopcount = loopcount + 1;
                        for (Recognition recognition : updatedRecognitions) {
                            mineralcounter = mineralcounter + 1;
                            telemetry.addData("Mineral counter", mineralcounter);
                            telemetry.addData("Label", recognition.getLabel());
                            //telemetry.addData("Left", (int) recognition.getLeft());
                            //telemetry.addData("Right", (int) recognition.getRight());
                            telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            //telemetry.addData("Height", recognition.getHeight());
                            telemetry.addData("Width", recognition.getWidth());
                            telemetry.addData("Bottom", recognition.getBottom());
                            //telemetry.addData("Top", recognition.getTop());
                            if (loopcount > 2 && recognition.getBottom() > closestbottom) {
                                closestbottom = recognition.getBottom();
                                closestmineral = mineralcounter;
                                closestlabel = recognition.getLabel();
                            }

                        }

                    }
                    else
                    {

                        strafeLeft(.15,300,100);
                        loopcount = 0;
                        noobjcount = 0;
                        closestmineral = 0;
                        closestbottom = 0;
                        closestlabel = "";
                        noobjcount=noobjcount+1;

                    }
                }
            }

            telemetry.addData("loopcount", loopcount);
            telemetry.addData("Closest Mineral", closestmineral);
            telemetry.addData("Closest Bottom", closestbottom);
            telemetry.addData("Closest Label", closestlabel);
            telemetry.update();
            sleep(300);
        }

        found = (closestbottom > 0 && closestlabel == "Gold Mineral");
        telemetry.addData("Found", found);

        telemetry.update();

        return found;
    }



    public boolean checktfod() {
        boolean found;
        int loopcount = 0;

        telemetry.update();  // clear out telemetry?
        int closestmineral = 0;
        double closestbottom = 0;
        String closestlabel = "";

        while (opModeIsActive() && loopcount < 5) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() > 0) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        int mineralcounter;
                        mineralcounter = 0;
                        closestmineral = 0;
                        closestbottom = 0;
                        loopcount = loopcount + 1;
                        for (Recognition recognition : updatedRecognitions) {
                            mineralcounter = mineralcounter + 1;
                            telemetry.addData("Mineral counter", mineralcounter);
                            telemetry.addData("Label", recognition.getLabel());
                            //telemetry.addData("Left", (int) recognition.getLeft());
                            //telemetry.addData("Right", (int) recognition.getRight());
                            telemetry.addData("Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            //telemetry.addData("Height", recognition.getHeight());
                            telemetry.addData("Width", recognition.getWidth());
                            telemetry.addData("Bottom", recognition.getBottom());
                            //telemetry.addData("Top", recognition.getTop());
                            if (loopcount > 2 && recognition.getBottom() > closestbottom) {
                                closestbottom = recognition.getBottom();
                                closestmineral = mineralcounter;
                                closestlabel = recognition.getLabel();
                            }

                        }

                    }
                }
            }

            telemetry.addData("loopcount", loopcount);
            telemetry.addData("Closest Mineral", closestmineral);
            telemetry.addData("Closest Bottom", closestbottom);
            telemetry.addData("Closest Label", closestlabel);
            telemetry.update();
            sleep(300);
        }

        found = (closestbottom > 0 && closestlabel == "Gold Mineral");
        telemetry.addData("Found", found);

        telemetry.update();

        return found;
    }









    public void deactivateTfod(){
        tfod.deactivate();
        vuforia = null;
    }

    private double getSpeed(double ticks_traveled) {
        double new_speed;

        if (gs_first_run) {
            gs_previous_ticks_traveled = ticks_traveled;
            gs_speed_timer.reset();
            gs_previous_speed = 1;
            gs_first_run = false;
        }

        if (gs_speed_timer.seconds() >= .1) {
            new_speed = (ticks_traveled - gs_previous_ticks_traveled) / 46.5;  // At max speed we travel about 4800 ticks in a second so this give a range of 0 - 10 for speed
            gs_speed_timer.reset();
            gs_previous_speed = new_speed;
            gs_previous_ticks_traveled = ticks_traveled;
            DbgLog.msg("10435 getspeed:" + Double.toString(new_speed));
        } else {
            new_speed = gs_previous_speed;
        }

        return new_speed;
    }

    private boolean checkfrontdistancesensor() {
        boolean wallfound = false;
        wallfound = true;
        // zno if (frontdistancesensor.getDistance(DistanceUnit.INCH) <= 22) {
            wallfound = true;
       // zno }

        return wallfound;
    }

    private double go_straight_adjustment(double target_heading) {

        //  This function outputs power_adjustment that should be added to right wheel and subtracted from left wheel

        double gs_adjustment;
        double current_heading;
        double degrees_off;
        boolean go_right;

        current_heading = currentheadingreading();

        DbgLog.msg("10435 Starting go_straight_adjustment heading:" + Double.toString(target_heading) + " current heading:" + current_heading);

        go_right = target_heading > current_heading;
        degrees_off = Math.abs(target_heading - current_heading);

        if (degrees_off > 180) {
            go_right = !go_right;
            degrees_off = 360 - degrees_off;
        }

        if (degrees_off < .3) {
            gs_adjustment = 0;
        } else {
            gs_adjustment = (Math.pow((degrees_off + 2) / 5, 2) + 2) / 100;
        }

        if (go_right) {
            gs_adjustment = -gs_adjustment;
        }

        DbgLog.msg("10435 ending go_straight_adjustment adjustment:" + Double.toString(gs_adjustment));

        return gs_adjustment;

    } // end of go_straight_adjustment

    public double currentheadingreading() {
        double current_heading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = angles.firstAngle;


        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);


        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }

        current_heading = shiftheading(current_heading);

        return current_heading;
    }

    private double shiftheading(double heading) {
        double shiftvalue = 3;
        heading = heading + shiftvalue;

        if (heading >= 360) {
            heading = heading - 360;
        } else if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }
   /* public void strafeRightBC(double power, int sleeptime, int endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(-power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }*/

    public void strafeRightBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(-power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void forwardBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void backwardBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void strafeLeftBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnRightBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnLeftBC(double power, long sleeptime, long endSleeptime) {
        sleeptime = getTimeBasedVoltage(sleeptime);
        endSleeptime = getTimeBasedVoltage(endSleeptime);
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }


    public void strafeRightBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(-power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void forwardBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void backwardBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void strafeLeftBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnRightBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }

    public void turnLeftBCP(double tpower, int sleeptime, int endSleeptime) {
        double power = getBatteryVoltage(tpower);
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        sleep(sleeptime);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(endSleeptime);
    }
}

