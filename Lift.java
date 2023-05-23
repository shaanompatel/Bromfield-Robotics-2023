/*
Created by 23spatel on 5/26/22
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Contains code to interface with lift in both user-controlled and autonomous modes
 * @author Shaan
 */
public class Lift {
    //hardware map
    private HardwareMap hwMap;

    // lift arm
    public DcMotor liftMotor;
    public Servo clawServo;
    
    // number of encoder clicks per centimeter (not accurate)
    private double clicksPerCm = 1.45/10;
    
    // default servo position (0-1) for the claw when robot initializes
    private double clawPos = 0.5;
    
    // encoder positions for the lift's different positions
    private int bottom, low, medium, high;
    
    // lift power when lift motor is in manual control
    private int liftPower = 0;
    
    // lift target position when lift motor is in preset mode
    private int liftTarget = 0;
 
    /**
     * Constructor
     * @param hwMap hardware map from the phone
    */
    public Lift(HardwareMap hwMap) {
        init(hwMap);
    }
    
    /**
     * initalizes devices from hardware map and configures motor options
     * @param hwMap hardware map from the phone
    */
    public void init(HardwareMap hwMap) {
        initHardwareMap(hwMap);
        hwMap.logDevices();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setPositions();
    }
    
    /**
     * gets current lift height and creates height presets
     * the four heights are bottom, low, medium, and high
     * the units are encoder clicks
    */
    public void setPositions(){
        bottom = liftMotor.getCurrentPosition();
        low = bottom + 10000/3;
        medium = bottom + 17000/3;
        high = bottom + 22500/3;
    }
    
    /**
    * Initializes the hardware map by defining lift motors
    * This method is only for the lift hardware
    * @param hwMap the hardware map from the phone
    */
    public void initHardwareMap(HardwareMap hwMap) {
        this.hwMap = hwMap;
        liftMotor = hwMap.dcMotor.get("liftMotor");
        clawServo = hwMap.servo.get("clawServo");
    }
    
    /**
     * Changes the liftTarget variable to the desired lift height
     * If multiple params are true, it defaults to the lowest height
     * @param bottom boolean value to bring lift to lowest height
     * @param low boolean value to bring lift to low juction height
     * @param medium boolean value to bring lift to middle juction height
     * @param high boolean value to bring lift to high juction height
    */
    public void setTargetLiftPosition(boolean bottom, boolean low, boolean medium, boolean high) {
        if (bottom){
            this.liftTarget = this.bottom;
        }
        else if(low){
           this.liftTarget = this.low; 
        }
        else if(medium){
            this.liftTarget = this.medium; 
        }
        else if(high){
            this.liftTarget = this.high;
        }
    }
    
    /**
     * changes the speed of the lift when it is in manual control (d-pad)
     * lift is either stationary or moving up/down at full speed
     * if both booleans are set to true, lift will default to up
     * @param up boolean value to enable upward movement
     * @param down boolean value to enable downward movement
    */
    public void setLiftPower(boolean up, boolean down) {
        if(up){
           liftPower = -1;
            
        }
        else if (down){
            liftPower = 1;
            
        }
        else {
            liftPower = 0;
            
        }
    }
    
    /**
     * allows for manual, granular adjustment of the lift height
     * lift moves at power <code>liftPower</code>
    */
    public void adjustHeight(){
       liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       liftMotor.setPower(liftPower);
    }
    
    /**
     * moves the lift to the height of the <code>liftTarget</code> instance variable
     * changes motor to run to position mode
     * overloaded method
    */
    public void goToPreset(){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goToPosition(this.liftTarget, 1);
    }
    
    
    public void goToPreset(boolean bottom, boolean low, boolean medium, boolean high){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       
        if (bottom){
            goToPosition(this.bottom, 1);
        }
        else if(low){
           goToPosition(this.low, 1); 
        }
        else if(medium){
            goToPosition(this.medium, 1); 
        }
        else if(high){
            goToPosition(this.high, 1);
        }
    }
    
    /**
     * raises lift to specified position with specified speed
     * @param pos desired lift height
     * @param speed desired lift speed
    */
    public void goToPosition(int pos, double speed){
        liftMotor.setTargetPosition((int)pos*-1);
        liftMotor.setPower(speed);
        
        // wait for move to complete
       // while (liftMotor.isBusy()) { //eventually eliminate while loop
            
       // }
        // Stop all motion;
        //liftMotor.setPower(0);
    }
    
    /**
     * opens and closes the claw
     * adjust servo position values as needed
     * servo will default to close if both params are true
     * @param open set to true in order to open the claw
     * @param close set to true in order to close the claw
    */
    public void setClawPos(boolean open, boolean close){
        if (close){
            clawServo.setPosition(0.6);
        }
        else if (open){
            clawServo.setPosition(0.2);
        }
    }
}