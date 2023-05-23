/*
    Created by 23spatel on 9/16/22
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * class for the robot's basic funtions
 * implements functionality of the robot's drive motors
*/
public class Drivetrain {
    
    // declare hardware map
    private HardwareMap hwMap;
    
    /**
     * variables for the drive motors
    */
    public DcMotor frmotor;
    public DcMotor flmotor;
    public DcMotor brmotor;
    public DcMotor blmotor;
    
    public BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction, rotation;

    private Telemetry telemetry;
    
    private double y, x, rotX, botHeading, rotY, denominator;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    private int initialPos;
    
    /**
     * values for calculating number of encoder clicks for certain distance
     * these values may need to be adjusted
    */
    private double clicksPerCm = 16; // empirically measured
    private double clicksPerDeg = 9.7; // empirically measured
    private double clicksPerCmStrafe = 18; // empirically measured

    public DistanceSensor rDistance, bDistance;
    double currentrDistance, currentbDistance;
    
    private int throttle = 2;
    private int liftMode = 0;

    /**
     * constructor
     * initializes the hardware map and sets motors to their starting state
     * @param hwMap the hardware map from the phone
    */
    public Drivetrain(HardwareMap hwMap){
        init(hwMap);
        resetMotors();
        stop();
    }
    
    /**
     * method to set up the hardware
     * @param hwMap the hardware map from the phone
    */
    private void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        hwMap.logDevices();
        initIMU();
    }
    
    /**
     * method to retrieve the devices from the hardware map
     * @param hwMap the hardware map from the phone
    */
    private void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        frmotor = hwMap.dcMotor.get("frontR");
        flmotor = hwMap.dcMotor.get("frontL");
        brmotor = hwMap.dcMotor.get("backR");
        blmotor = hwMap.dcMotor.get("backL");
        rDistance = hwMap.get(DistanceSensor.class, "rDistance");
        bDistance = hwMap.get(DistanceSensor.class, "bDistance");
    }
    
    /**
     * method to initialize the IMU
     * uses values from the IMU instance variables 
    */
    public void initIMU(){
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
    }
    
    /**
     * method to stop the drive motors and reset them
    */
    public void stop() {
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);
        resetMotors();
    }
    
    /**
     * method to reset the motor settings
     * the motors default to run without encoders
     * reverses motors as needed
    */
    public void resetMotors() {
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        flmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    
    /**
     * method to configure the motors for autonomous control
     * sets the motors to run to position mode
     * configures the motors so that they brake if power is set to 0
    */
    public void setAuto(){
        flmotor.setTargetPosition(flmotor.getCurrentPosition());
        frmotor.setTargetPosition(frmotor.getCurrentPosition());
        brmotor.setTargetPosition(brmotor.getCurrentPosition());
        blmotor.setTargetPosition(blmotor.getCurrentPosition());
       
        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * method to set motor power using human joystick input
     * supports moving in the y-direction, rotating, strafing, and a combination 
     * @param y y-coordinate of left joystick
     * @param x x-coordinate of left joystick
     * @param rx x-coordinate of right jotstick
     * @param ry y-coordinate of right joystick
     * @param heading robot heading from the IMU
    */
    public void useJoystick(double y, double x, double rx, double ry, double heading){
        // Read reverse IMU heading, as the IMU heading is CW positive
        botHeading = -1*heading;

        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
        move(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    
    /**
     * method that allows drive motor control using gamepad triggers
     * only supports movement in the y-direction
     * @param left desired reverse motor power
     * @param right desired foward motor power
    */
    public void useTrigger(double left, double right){
        if (right > 0){
            move(right);
        }
        else if (left > 0){
            move(left*-1);
        }
    }
    
    /**
     * helper method that sets the motor powers
     * throttle is a changeable instance variable to regulate the speed
     * @param flpower motor power for front-left motor
     * @param frpower motor power for front-right motor
     * @param blpower motor power for back-left motor
     * @param brpower motor power for back-right motor
    */
    private void move(double flpower, double frpower, double blpower, double brpower){
        flmotor.setPower(flpower/throttle);
        frmotor.setPower(frpower/throttle);
        blmotor.setPower(blpower/throttle);
        brmotor.setPower(brpower/throttle);
    }
    
    /**
     * overloaded helper method that sets all motor powers to one value
     * @param power desired power for all four motors
    */
    public void move(double power){ //temp public
        flmotor.setPower(power/throttle);
        frmotor.setPower(power/throttle);
        blmotor.setPower(power/throttle);
        brmotor.setPower(power/throttle);
    }
    
    /**
     * method to move the robot forward/backward a certain distance
     * @param distance the desired number of centimeters
     * @param speed speed of the motors – negative value goes backwards
    */
    public void moveForward(int distance, double speed) {
        // fetch motor positions
        int flPos = flmotor.getCurrentPosition();
        int frPos = frmotor.getCurrentPosition();
        int blPos = blmotor.getCurrentPosition();
        int brPos = brmotor.getCurrentPosition();
        
        initialPos = flPos;

        // calculate new targets
        flPos += distance * clicksPerCm;
        frPos += distance * clicksPerCm;
        blPos += distance * clicksPerCm;
        brPos += distance * clicksPerCm;
        
        execute(flPos, frPos, blPos, brPos, speed);
    }
    
    /**
     * rotate the robot clockwise or counterclockwise
     * aka spin in place
     * not currently used in autonomous
     * @param left rotate counterclockwise
     * @param right rotate clockwise
    */
    public void rotate(boolean left, boolean right){
        if (left){
            move(0.8, -0.8, 0.8, -0.8);
        }
        else if (right){
            move(-0.8, 0.8, -0.8, 0.8);
        }
    }
    
    /**
     * strafe the robot left or right
     * @param left strafe left
     * @param right strafe right
    */
    public void strafe(boolean left, boolean right){
        if (left){
            move(2, -2, -2, 2);
        }
        else if (right){
            move(-2, 2, 2, -2);
        }
    }
    
    /**
     * move forward or backward
     * adjust power as needed
     * @param left (left trigger) move backward
     * @param right (right trigger) move forward
    */
    public void forward(boolean left, boolean right){
        if (left){
            move(-1);
        }
        else if (right){
            move(1);
        }
    }

    /**
     * rotate a specified number of degrees at a specified speed
     * @param the number of degrees clockwise. Negative is counterclockwise
     * @param speed speed of rotation. Varying the speed can mess with the angle
    */
    public void turnRight(int angle, double speed) {
        // angle is in degrees. A negative angle turns counterclockwise.

        // fetch motor positions
        int flPos = flmotor.getCurrentPosition();
        int frPos = frmotor.getCurrentPosition();
        int blPos = blmotor.getCurrentPosition();
        int brPos = brmotor.getCurrentPosition();

        // calculate new targets
        flPos += angle * clicksPerDeg;
        frPos -= angle * clicksPerDeg;
        blPos += angle * clicksPerDeg;
        brPos -= angle * clicksPerDeg;

        execute(flPos, frPos, blPos, brPos, speed);
    }
    
    /**
     * rotates the robot a specified number of degrees at a sepecified speed
     * @param degrees number of degrees counterclockwise. Negative is clockwise
     * @param speec speed of rotation. Varying the speed can mess with the angle
    */ 
    public void turnLeft(int degrees, double speed){
        turnRight(-degrees, speed);
    }
    
    /**
     * strafes the robot a certain distance in cm
     * @param distance distance to be moved left. Negative is right
     * @param speed speed of motors
    */
    public void moveLeft(int distance, double speed){
        moveRight(-distance, speed);
    }
    
    /**
     * strafes the robot a certain distance in cm
     * @param distance distance to be strafed right. Negative is left
     * @param speed speed of motors
    */
    public void moveRight(int distance, double speed) {
        // distance is in centimeters. A negative distance moves backward.
        // fetch motor positions
        int flPos = flmotor.getCurrentPosition();
        int frPos = frmotor.getCurrentPosition();
        int blPos = blmotor.getCurrentPosition();
        int brPos = brmotor.getCurrentPosition();
        // calculate new targets
        flPos += distance * clicksPerCmStrafe;
        frPos -= distance * clicksPerCmStrafe;
        blPos -= distance * clicksPerCmStrafe;
        brPos += distance * clicksPerCmStrafe;
        
        execute(flPos, frPos, blPos, brPos, speed);
    }
    
    /**
     * helper method to move the robot using encoders
     * @param flPos desired position of the front-left motor
     * @param frPos desired position of the front-right motor
     * @param blPos desired position of the back-left motor
     * @param brPos desired position of the back-right motor
     * @param speed speed of the motors
     * 
    */
    public void execute(int flPos, int frPos, int blPos, int brPos, double speed){
        // move robot to new position
        flmotor.setTargetPosition(flPos);
        frmotor.setTargetPosition(frPos);
        blmotor.setTargetPosition(blPos);
        brmotor.setTargetPosition(brPos);
        move(speed);

        // wait for move to complete
        while (flmotor.isBusy() && frmotor.isBusy() &&
                blmotor.isBusy() && brmotor.isBusy()) {
                //if (brPos > 0.5*initialPos){
                  //  speed /= 2;
                    //move(speed);
                    //initialPos = flmotor.getCurrentPosition();
               // }
        }

        // Stop all motion;
        move(0);
    }
    
    /**
     * uses ultrasonic sensor to move robot vertically until it is a specified disance from a wall
     * method does not work perfectly
     * @param bDist desired distance away from back wall
    */
    public void orientVertical(double bDist) throws InterruptedException{ 
        //get bDistance
        currentbDistance  = getbDistance();
        
        while (Math.abs(currentbDistance - bDist) > 1){
            if(currentbDistance > (bDist - 1)){
                move(-1);
            }
            if(currentbDistance < (bDist + 1)){
                move(1);
            }
            currentbDistance = bDistance.getDistance(DistanceUnit.CM);
        } 
        
        move(0);
    }
    
    /**
     * uses ultrasonic sensor to move robot horizontally until it is a specified disance from a wall
     * method does not work perfectly
     * @param rDist desired distance away from right wall
    */
    public void orientHorizontal(double rDist) throws InterruptedException{ //changed so error is +-1
        currentrDistance = getrDistance();
        while (Math.abs(currentrDistance - rDist) > 1){
            if(currentrDistance > (rDist - 1)){
                move(1, -1, -1, 1);  //strafe
            }
            if(currentrDistance < (rDist + 1)){
                move(-1, 1, 1, -1);  //strafe
            }
            currentrDistance = rDistance.getDistance(DistanceUnit.CM);
        } 
        move(0);
    }
    
    /**
     * gets robot's distance from wall to the right 
    */ 
    public Double getrDistance() throws InterruptedException{
        currentrDistance = 0;
        
        for (int i = 0; i < 10; i++)
        {
            currentrDistance += rDistance.getDistance(DistanceUnit.CM);
            wait1(100);
        }
        currentrDistance /= 10;
        
        return (currentrDistance);
    }
    
    /**
     * gets robot's distance from wall in the back
    */
    public Double getbDistance() throws InterruptedException{
        currentrDistance = 0;
        
        for (int i = 0; i < 10; i++){
            currentrDistance += rDistance.getDistance(DistanceUnit.CM);
            wait1(100);
        }
        currentrDistance /= 10;
        
        return (currentbDistance);
    }
    
    /**
     * toggles the speed instance variable to increase/decrease motor power
     * does not work perfectly
    */ 
    public void changeSpeed(boolean speed){
        if (speed){
            if (throttle == 2){
                throttle = 4;
            }
            else if (throttle == 4){
                throttle = 2;
            }
        }
    }
    
    
    /**
     * disregard this code
     * allows the robot to be driven completely with one method
     * has a bug
    */ 
    public void drive
    (
        Drivetrain robot, Lift coneLift, boolean gamepad2A, 
        double gamepad2LT, double gamepad2RT, double gamepad2LSY, double gamepad2LSX, 
        double gamepad2RSX, boolean gamepad2DPR, boolean gamepad2DPL, boolean gamepad2RB,
        boolean gamepad2LB, boolean gamepad1A, boolean gamepad1B, boolean gamepad1Y, boolean gamepad1X,
        boolean gamepad1LB, boolean gamepad1RB, boolean gamepad1DPU, boolean gamepad1DPD,
        BNO055IMU imu
    )
    
    {
        robot.changeSpeed(gamepad2A);
        if (gamepad2LT != 0 ||gamepad2RT != 0){
            robot.useTrigger(gamepad2LT, gamepad2RT);
        }

        else {
            robot.useJoystick(-1*gamepad2LSY, gamepad2LSX, gamepad2RSX, gamepad2LSX, imu.getAngularOrientation().firstAngle);
        }
            
        if (gamepad2DPR || gamepad2DPL){
              robot.rotate(gamepad2DPR, gamepad2DPL);
        }
            
        if (gamepad2RB|| gamepad2LB){
            robot.strafe(gamepad2RB, gamepad2LB);
        }
            
        if(gamepad1LB || gamepad1RB){
            coneLift.setClawPos(gamepad1RB, gamepad1LB);
        }
        
        if (gamepad1DPU || gamepad1DPD){
            liftMode = 0;
            coneLift.setLiftPower(gamepad1DPU, gamepad1DPD);
            //coneLift.adjustHeight(gamepad1.dpad_up, gamepad1.dpad_down);
        }
        else {
            coneLift.setLiftPower(false, false);
        }
            
        if (gamepad1A || gamepad1B || gamepad1X || gamepad1Y) {
            liftMode = 1;
            coneLift.setTargetLiftPosition(gamepad1A, gamepad1B, gamepad1Y, gamepad1X);
        }
            
        if (!(coneLift.liftMotor.isBusy())) {
            coneLift.liftMotor.setPower(0);
        }
        
        if (liftMode == 0) {
                coneLift.adjustHeight();
        }
            
        if (liftMode == 1) {
                coneLift.goToPreset();
        } 
    }

    /**
     * pauses execution of the program
     * @param t number of millisecond to pause
    */
    private void wait1(int t) throws InterruptedException {
        TimeUnit.MILLISECONDS.sleep(t);
    }
}




