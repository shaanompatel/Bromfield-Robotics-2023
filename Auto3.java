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

public class Auto3 extends LinearOpMode {
    
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
            sleep(1000);
            coneLift.goToPreset(false, false, false, true);
            
            while (coneLift.liftMotor.isBusy()) {
                sleep(10);
            }

            // move to stack
            robot.moveLeft(40, 0.5);
            robot.moveForward(-3, 0.5);
            sleep(500);
            
            
            // make prediction
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

            robot.moveForward(-7, 0.5);
            sleep(200);
            
            // push cone out of the way
            robot.moveLeft(119, 0.5);
            sleep(100);
            robot.moveForward(-5,0.5);
            sleep(200);
            robot.turnLeft(10, 0.5);
            sleep(200);
            
            //robot.moveRight(34, 0.5);
            
            //robot.turnLeft(5, 0.5);
            
            robot.moveForward(11, 0.5);
            sleep(200);
            coneLift.setClawPos(false, true);
            
            sleep(500);
            
            
            //sleep(70);
            //robot.moveForward(-7, 0.5);
            //robot.moveRight(30, 0.5);
            
            // move to correct parking spot
           // if (prediction == 1){
            //    robot.moveForward(-58, 0.5);
 
//            }
  //          else if (prediction >= 3){
    //            robot.moveForward(65, 0.5);
      //      }
            
        //    coneLift.goToPreset(true, false, false, false);
            
            while (coneLift.liftMotor.isBusy()) {
                sleep(10);
            }
        }
    }
}
