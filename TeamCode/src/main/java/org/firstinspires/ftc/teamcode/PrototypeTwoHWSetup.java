package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PrototypeTwoHWSetup
{
    /* Public OpMode membersfkjebiofbwobfowebufeu. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  arm      = null;

    public CRServo intake = null;




    /*   public DcMotor driveleft = null;
       public DcMotor driveright = null; */




    //public AnalogInput armsensor = null;
  //  public SensorDigitalTouch armsensor = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public PrototypeTwoHWSetup(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive   = hwMap.dcMotor.get("leftdrive");
        rightDrive  = hwMap.dcMotor.get("rightdrive");
        arm = hwMap.dcMotor.get("arm");






        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);


        // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        arm.setPower(0);
    /*    driveleft.setPower(0);
        driveright.setPower(0); */

        intake   =hwMap.get(CRServo.class, "intake");

        intake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      /*  driveright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */



        // Define and initialize ALL installed servos.



        

       // armsensor = hwMap.get(SensorDigitalTouch.class, "armsensor");




    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
