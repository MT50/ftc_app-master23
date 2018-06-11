package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by Computer on 18/08/2017.
 */

@TeleOp(name = "Gyro")
public class Gyro extends OpMode {
    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
@Override
    public void init(){
    RF=hardwareMap.dcMotor.get("RF");
    RB=hardwareMap.dcMotor.get("RB");
    LB=hardwareMap.dcMotor.get("LB");
    LF=hardwareMap.dcMotor.get("LF");
    modernRoboticsI2cGyro=hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
    gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
    modernRoboticsI2cGyro.calibrate();
}

    @Override
    public void loop() {
        //hey ho
      //  AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        double zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
double forward=gamepad1.left_stick_y;
        double right=gamepad1.left_stick_x;
        double clockwise=gamepad1.right_stick_x;
        double temp=forward*Math.cos(zAngle)+right*Math.sin(zAngle);
         right=forward*Math.sin(zAngle)+right*Math.cos(zAngle);
        forward=temp;
        double frontl=forward+clockwise+right;
        double frontr=forward-clockwise-right;
        double rearl=forward+clockwise-right;
        double rearr=forward-clockwise+right;
        double max=Math.abs(frontl);
        if(Math.abs(frontr)>max)
            max=Math.abs(frontr);
        if(Math.abs(rearr)>max)
            max=Math.abs(rearr);
        if(Math.abs(rearl)>max)
            max=Math.abs(rearl);
        if(max>1)
        {
            frontl/=max;
            frontr/=max;
            rearl/=max;
            rearr/=max;
        }

        RF.setPower(DF());
        RB.setPower(rearr);
        LB.setPower(rearl);
        LF.setPower(RB.hashCode());
    }
static int DF(){
    return 2;
}
}
