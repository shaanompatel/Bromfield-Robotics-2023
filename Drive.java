package org.firstinspires.ftc.teamcode;
/**
 * Created by 23spatel on 5/26/22.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drivetrain", group="Drive")
public class Drive extends LinearOpMode {

    private double y, x, botHeading, rotX, rotY, denominator;
    private double frontLeftPower, backRightPower, frontRightPower, backLeftPower;
    private double clawPos;
    private double liftPos;
    private Drivetrain robot;
    private BNO055IMU imu;
    private Lift coneLift; 
    private BNO055IMU.Parameters parameters;
    private int liftMode = 0;
    
    @Override
    public void runOpMode(){
        //Initialize the hardware variables.
 
        robot = new Drivetrain(hardwareMap);
        coneLift = new Lift(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        clawPos = 1.0;
        liftPos = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("MSG", "Robot init");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           
            robot.changeSpeed(gamepad2.a);
            if (gamepad2.left_trigger != 0 ||gamepad2.right_trigger != 0){
                robot.useTrigger(gamepad2.left_trigger, gamepad2.right_trigger);
            }

            else {
                robot.useJoystick(-1*gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, gamepad2.left_stick_x, imu.getAngularOrientation().firstAngle);
            }
            
            if (gamepad2.dpad_right || gamepad2.dpad_left){
               robot.rotate(gamepad2.dpad_right, gamepad2.dpad_left);
            }
            
            if (gamepad2.right_bumper || gamepad2.left_bumper){
                robot.strafe(gamepad2.right_bumper, gamepad2.left_bumper);
            }
            
            if (gamepad1.dpad_up || gamepad1.dpad_down){
                liftMode = 0;
                coneLift.setLiftPower(gamepad1.dpad_up, gamepad1.dpad_down);
                //coneLift.adjustHeight(gamepad1.dpad_up, gamepad1.dpad_down);
            }
            else {
                coneLift.setLiftPower(false, false);
            }
            
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                liftMode = 1;
                coneLift.setTargetLiftPosition(gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x);
                coneLift.goToPreset(gamepad1.a, gamepad1.b, gamepad1.y, gamepad1.x);
            }
            
            if(gamepad1.left_bumper || gamepad1.right_bumper){
                coneLift.setClawPos(gamepad1.right_bumper, gamepad1.left_bumper);
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
            
            //coneLift.goToPreset(true,true,true,true);

        
            telemetry.addData("bottom", gamepad1.a);
            telemetry.addData("low", gamepad1.b);
            telemetry.addData("rightTrigger ", gamepad2.right_trigger);
            telemetry.addData("leftTrigger ", gamepad2.left_trigger);
            telemetry.addData("dpad", gamepad2.dpad_right);
            telemetry.addData("lift dpad", gamepad1.dpad_up);
            telemetry.addData("liftMode", liftMode);
            telemetry.update();

             

        }
        
 
    }
        
}
