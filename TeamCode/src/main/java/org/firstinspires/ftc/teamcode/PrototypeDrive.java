/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Pushbot: PrototypeDrive", group="Pushbot")
//@Disabled
public class PrototypeDrive extends OpMode{
   /* int relicdistace;
      boolean relicclaw;
    /* Declare OpMode members. */
    PrototypeHWSetup robot = new PrototypeHWSetup();// use the class created to define a Pushbot's hardware
     double reverse = 1;
     double reversem = 1;
     double speed = 1;

//----------------------------------------------------------------------
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle = 0;
    double turning;
    double mecanum;
    double negmecanum;
//----------------------------------------------------------------------


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
      //  relicclaw = false;

//----------------------------------------------------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated())
        {

        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
//----------------------------------------------------------------------

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }
    /*
     * Code to run ONCE when the driver hits PLAY
     *
     */


    @Override
    public void start() { }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
/*
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;


            robot.frontleftMotor.setPower(v1 * v1 * v1 * 5);
            robot.frontrightMotor.setPower(v2 * v2 * v2 * 5 );
            robot.backleftMotor.setPower(v3 * v3 * v3 * 5);
            robot.backrightMotor.setPower(v4 * v4 * v4 * 5);
*/
        turning = gamepad1.left_stick_x / 2;
        negmecanum = gamepad1.right_stick_y - gamepad1.right_stick_x;
        mecanum = gamepad1.right_stick_y + gamepad1.right_stick_x;

        if (gamepad1.a) {
            getAngle();
            if (globalAngle > 0){
                turning += .5;
            }
            else{
                turning -= .5;
            }
        }

        robot.leftFrontDrive.setPower((((negmecanum)* reverse)  - (turning)) * speed);
        robot.rightBackDrive.setPower((((negmecanum)* reverse)  + (turning)) * speed);
        robot.rightFrontDrive.setPower((((mecanum)* reverse) + (turning)) * speed);
        robot.leftBackDrive.setPower(((((mecanum) * reverse) - (turning)) * speed));

        robot.arm.setPower(gamepad2.right_stick_y * gamepad2.right_stick_y * gamepad2.right_stick_y *gamepad2.right_stick_y *gamepad2.right_stick_y / 2 );

        robot.intake.setPower(-gamepad2.left_stick_y);

        robot.lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

        if (gamepad2.a){
            robot.door.setPosition(.03);
        }
        if (gamepad2.x){
            robot.door.setPosition(.45);
        }

        if (gamepad2.left_bumper){
            robot.mineralarm.setPower(-1);
        }
        else if (gamepad2.right_bumper){
            robot.mineralarm.setPower(1);
        }
        else {
            robot.mineralarm.setPower(0);
        }

        if (gamepad1.left_bumper){
            speed = .5;
        }
        else {
            speed = 1;
        }


        if (gamepad1.right_bumper){
            if (reversem == 1) {
                reversem = -1;
                if (reverse > 0){
                    reverse = -1;
                }
                else {
                    reverse = 1;
                }
            }

        }
        else {
            reversem = 1;
        }

        if (gamepad1.x){
            resetAngle();
        }

        telemetry.addData("drive 1" ,(robot.leftFrontDrive.getCurrentPosition()));
//----------------------------------------------------------------------
        getAngle();
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.update();
//----------------------------------------------------------------------
    }



    //----------------------------------------------------------------------
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    //----------------------------------------------------------------------

    @Override
    public void stop() {
    }

}
