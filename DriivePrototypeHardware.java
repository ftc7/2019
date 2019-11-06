package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriivePrototypeHardware {
    public DcMotor one = null;
    public DcMotor two = null;
    public DcMotor three = null;
    public DcMotor four = null;

    //public CRServo servo = null;

    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public float dangles[] = new float[3];

    HardwareMap hwMap = null;

    public DriivePrototypeHardware(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        one = ahwMap.dcMotor.get("one");
        two = ahwMap.dcMotor.get("two");
        three = ahwMap.dcMotor.get("three");
        four = ahwMap.dcMotor.get("four");

        //servo = ahwMap.crservo.get("servo");

        /*three.setDirection(DcMotor.Direction.REVERSE);
        four.setDirection(DcMotor.Direction.REVERSE);*/

        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        three.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        four.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        dangles[0] = angles.firstAngle;
        dangles[1] = angles.secondAngle;
        dangles[2] = angles.thirdAngle;
    }

    public void updateGyro(int damping) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        /*dangles[0] = ((dangles[0] * damping) + angles.firstAngle) / (damping + 1);
        dangles[1] = ((dangles[1] * damping) + angles.secondAngle) / (damping + 1);
        dangles[2] = ((dangles[2] * damping) + angles.thirdAngle) / (damping + 1);*/

        gravity = imu.getGravity();
    }
}