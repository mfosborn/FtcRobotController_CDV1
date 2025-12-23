package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name="Auto Blue Close", group="Robot")

public class autoBlueClose extends LinearOpMode {
    

    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    // private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20240210_071306.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    //private static final String[] LABELS = {
     //  "cube"
   // };
/**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
   // private VisionPortal visionPortal;
    /* Declare OpMode members. */
    private DcMotor bottomRight; //Right Rear Actual , port0
    private DcMotor topRight; //Right Front Actual, port1
    private DcMotor topLeft; //Left Front Actual, port2
    private DcMotor bottomLeft; //Left Rear Actual, port3
    private DcMotor motor5; //shooter 1!!!!!
    private DcMotor motor6; //shooter 2!!!!!
    private DcMotor motor7; //shooting angler
    private DcMotor motor8; //intake / tage
    private Servo servo1; //the ball whacker
    private Servo servo2; //the ball juggler
    
    private IMU imu = null; // Control/Expansion Hub IMU

    private double headingError = 0;
    double x = 0;
    double xCount = 0;
    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0.2;
    private double negSpeed = 0;
    private double turnSpeed = 0.1;
    private double Speed1 = 0;
    private double Speed2 = 0;
    private double Speed3 = 0;
    private double Speed4 = 0;
    private int Target1 = 0;
    private int Target2 = 0;
    private int Target3 = 0;
    private int Target4 = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 383.6 ; // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78 ; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.51; // Normal driving speed
    static final double DRIVEFAST_SPEED = 0.8; // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.4; // Max Turn speed to limit turn rate
    static final double SIDE_SPEED = 0.6; // Max Side speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() {

  //  initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Initialize the drive system variables.
        bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        //ballSorter = hardwareMap.get(DcMotor.class, "ballSorter");
        motor5 = hardwareMap.get(DcMotorEx.class, "motor5");
        motor6 = hardwareMap.get(DcMotorEx.class, "motor6");
        motor7 = hardwareMap.get(DcMotor.class, "motor7");
        motor8 = hardwareMap.get(DcMotor.class, "motor8");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        
        bottomRight.setDirection(DcMotor.Direction.REVERSE); //Right Rear Actual
        topRight.setDirection(DcMotor.Direction.REVERSE); //Right Front Actual
        topLeft.setDirection(DcMotor.Direction.FORWARD); //Left Front Actual
        bottomLeft.setDirection(DcMotor.Direction.FORWARD); //Left Rear Actual
        //ballSorter.setDirection(DcMotor.Direction.REVERSE); //ball sorter motor
        motor5.setDirection(DcMotor.Direction.REVERSE);
        motor6.setDirection(DcMotor.Direction.FORWARD); //shooter1
        motor7.setDirection(DcMotor.Direction.FORWARD); //shooter2

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; //orig right
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT; //orig up
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start 
        motor7.setTargetPosition(0);
        motor7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor7.setPower(1.0);

        
        //ballSorter.setTargetPosition(0);
        //ballSorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ballSorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ballSorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ballSorter.setPower(0.1);
        
                
        waitForStart();

        // Set the encoders for closed loop speed control, and reset the heading.
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();
      //  telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();


      //  holdHeading( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second


        if (opModeIsActive()) {
            while (opModeIsActive()) {

               // telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();
                /*
                sleep(1000);
                driveStraight(DRIVE_SPEED, -2.5, 0.0);
                sleep(1000);
                // servo_1.setPosition(0.3);//close claw
                // servo_4.setPosition(1);
                sleep(1000);
                turnToHeading( TURN_SPEED, 90.0);               // Turn  CW to -45 Degrees
                sleep(1000);
                driveSideways(0.1, 6, 90);
                sleep(1000);
                motor_5.setTargetPosition(1050);
                sleep(1000);
                driveStraight(); */
                //turnToHeading( TURN_SPEED, 50.0);               // Turn  CW to -45 Degrees
                //sleep(10000);
                driveStraight(DRIVE_SPEED, -50, 0.0);
                sleep(500);
                setAngleHigh();
                sleep(2000);
                shootClose();
                sleep(2000);
                //artifactArmBack();
                //sleep(500);
                setArtifact();
                sleep(2000);
                shootOff();
                sleep(500);
                turnToHeading(.6, -135);
                sleep(20000);
            }
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    //*****15786 functions for 2025 season
        
    public void setAngleLow()
    {
      motor6.setTargetPosition(10);
    }
    
    public void setAngleMed()
    {
      motor6.setTargetPosition(30);
    }
    
    public void setAngleHigh()
    {
        motor7.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor7.setTargetPosition(60);
    }

    public void setArtifact()
    {
        servo1.setPosition(.85);
    }

    public void artifactArmBack()
    {
        servo1.setPosition(.6);
    }

    public void setNextArtifact()
    {

    }
    
    public void shootClose()
    {
        motor5.setPower(.5);
        motor6.setPower(.5);
    }
    
    public void shootFar()
    {
        motor5.setPower(.8);
        motor6.setPower(.8);
    }
    
    public void shootOff()
    {
        motor5.setPower(0);
        motor6.setPower(0);
    }
    
        public void clawClosed()
    {
        //servo_3.setPosition(0.62); // closed right
      //servo_4.setPosition(0.61); //closed left
    }
    
    public void extendArm()
    {
                //servo_6.setPosition(.5); //this is angles to drop into basket
                 //servo_7.setPosition(.9); //claw tilter .5 orig setting
    }
    public void returnArm()
    {
                //servo_6.setPosition(1); //right .04
                 //servo_7.setPosition(0); //claw tilter .5 orig setting
    }
    
    public void releaseSpecimen()
    {
                //servo_4.setPosition(0.4); //left
                 //servo_3.setPosition(0.8); //right
                 //servo_7.setPosition(.3);
    }
    
    // **********  HIGH Level driving functions.  ********************

    /**
    *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the OpMode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            Target1 = bottomRight.getCurrentPosition() + moveCounts;
            Target2 = topRight.getCurrentPosition() + moveCounts;
            Target3 = topLeft.getCurrentPosition() + moveCounts;
            Target4 = bottomLeft.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            bottomRight.setTargetPosition(Target1);
            topRight.setTargetPosition(Target2);
            topLeft.setTargetPosition(Target3);
            bottomLeft.setTargetPosition(Target4);

            bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (bottomRight.isBusy() && topRight.isBusy() && topLeft.isBusy() && bottomLeft.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
  public void driveSideways(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            Target1 = bottomRight.getCurrentPosition() + moveCounts;
            Target2 = topRight.getCurrentPosition() - moveCounts;
            Target3 = topLeft.getCurrentPosition() + moveCounts;
            Target4 = bottomLeft.getCurrentPosition() - moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            bottomRight.setTargetPosition(Target1);
            topRight.setTargetPosition(Target2);
            topLeft.setTargetPosition(Target3);
            bottomLeft.setTargetPosition(Target4);

            bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobotSide(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (bottomRight.isBusy() && topRight.isBusy() && topLeft.isBusy() && bottomLeft.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                

                // Apply the turning correction to the current driving speed.
              //  moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobotSide(0, 0);
            bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        Speed1  = drive - turn;
        Speed2  = drive - turn;
        Speed3  = drive + turn;
        Speed4  = drive + turn;
        
        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(Speed1), Math.abs(Speed4));
        if (max > 1.0)
        {
            Speed1 /= max;
            Speed2 /= max;
            Speed3 /= max;
            Speed4 /= max;
        }

        bottomRight.setPower(Speed1);
        topRight.setPower(Speed2);
        topLeft.setPower(Speed3);
        bottomLeft.setPower(Speed4);

    }
    public void moveRobotSide(double drive, double turn) {
        double negDrive; 
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
        negDrive= drive;
        Speed1  = drive;
        Speed2  = drive;
        Speed3  = drive;
        Speed4  = drive;
        
        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(Speed1), Math.abs(Speed4));
        if (max > 1.0)
        {
            Speed1 /= max;
            Speed2 /= max;
            Speed3 /= max;
            Speed4 /= max;
        }

        bottomRight.setPower(Speed1);
        topRight.setPower(Speed2);
        topLeft.setPower(Speed3);
        bottomLeft.setPower(Speed4);

    }
    
    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      Target1,  Target2, Target3, Target4);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      bottomRight.getCurrentPosition(),
                    topRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", Speed1, Speed2, Speed3, Speed4);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
