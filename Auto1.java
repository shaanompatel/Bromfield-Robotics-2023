package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class Auto1 extends LinearOpMode {
    
    private Drivetrain robot; 
    private Detector sleeveReader; 
    private Lift coneLift;

    public void runOpMode() throws InterruptedException { 
        robot = new Drivetrain(hardwareMap);
        coneLift = new Lift(hardwareMap);
        robot.setAuto();
        
        waitForStart();
       
        if (opModeIsActive()) {
            // grasp and lift up cone
            coneLift.setClawPos(true, false);
            sleep(2000);
            
            // lift claw to low junction height
            coneLift.goToPreset(false, true, false, false);

            // move to lowest junction
            robot.moveLeft(34, 0.5);
            sleep(200);
            
            // add adjustment here if needed
            
            // move forward, drop the cone, and move backward
            robot.moveForward(7, 0.5);
            coneLift.setClawPos(false, true);
            robot.moveForward(-9, 0.5);
            
            // adjust angle so that robot is straight
            robot.moveLeft(7, 0.5);
            
            // lower the claw to lowest position
            coneLift.goToPreset(true, false, false, false);
            //robot.moveLeft(5, 0.5);
            sleep(500);
            
            // make prediction using phone camera
            sleeveReader = new Detector(hardwareMap);
            sleep(3000);
            long start = System.currentTimeMillis();
            long end = start + (3 * 1000);
            
            while(System.currentTimeMillis() < end){
                sleeveReader.drawBox();
            }
            
            int prediction  = sleeveReader.getPrediction();
            telemetry.addData("Prediction", prediction);
            telemetry.update();


            // push cone out of the way
            robot.moveLeft(55, 0.5);
            sleep(500);
            robot.moveRight(34, 0.5);
            
            // adjust angle so that robot is straight
            robot.turnLeft(5, 0.5);
            
            // move to correct parking spot
            if (prediction == 1){
                robot.moveForward(-55, 0.5);
 
            }
            else if (prediction >= 3){
                robot.moveForward(65, 0.5);
            }
            
        }
        
    }
}
