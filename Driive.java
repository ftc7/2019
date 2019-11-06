package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Driive {
    private DcMotor[] wheels;
    private double[] wheelAngles;
    private double zero;
    private double r;
    private double theta;
    private double currentAngle;
    private double setAngle;
    private double turn;
    private boolean turning = false;

    private double[] wheelPowers;

    /**
     * Marks whether the robot's driving is field centric
     * Defaults to true
     */
    public boolean fieldCentric = true;

    /**
     * Marks whether the robot fights to maintain its orientation
     * Defaults to false
     */
    public boolean righteous = false;

    /**
     * Speed, works on a scale of 1-10
     * Affects the overall speed of the robot
     */
    public int speed = 10;

    /**
     * Initializes the Driive instance
     * Called once in init
     *
     * @param wheels An array of the DcMotors to be used for driving
     * @param wheelAngles An array of the physical angles of the wheels, e.g. 0,90,180,270
     * @throws java.lang.ArrayIndexOutOfBoundsException - if wheels[] and wheelAngles[] are different lengths
     */
    public void init(DcMotor[] wheels, double[] wheelAngles) throws Exception{
        init(wheels, wheelAngles, 0);
    }

    /**
     * Initializes the Driive instance
     * Called once in init
     *
     * @param wheels An array of the DcMotors to be used for driving
     * @param wheelAngles An array of the physical angles of the wheels, e.g. 0,90,180,270
     * @param zero The starting angle of the robot; can be adjusted to change the initial "front"
     * @throws java.lang.Exception - if wheels[] and wheelAngles[] are different lengths
     */
    public void init(DcMotor[] wheels, double[] wheelAngles, double zero) throws Exception{
        if(wheels.length != wheelAngles.length) {
            throw new java.lang.Exception("Wheels[] and wheel angles[] are different lengths; please check your init code");
        }
        this.wheels = wheels;
        this.wheelAngles = wheelAngles;
        this.zero = zero;
        this.wheelPowers = new double[wheels.length];
    }

    /**
     * Control movement using x,y values
     * Call when movement should change
     *
     * @param x The X value of the joystick
     * @param y The Y value of the joystick
     */
    public void cartesian(double x, double y, double turn) {
        r = Math.sqrt(x*x+y*y);
        theta = Math.atan2(x, y);
        this.turn = turn;
    }

    /**
     * Control movement using polar coordinates
     * Call when movement should change
     *
     * @param r The magnitude of the vector
     * @param theta The angle of the vector
     */
    public void polar(double r, double theta, double turn) {
        this.r = r;
        this.theta = theta;
        this.turn = turn;
    }

    /**
     * Sets the gyroscope value
     * Call every loop with updated values
     *
     * @param angle The angle obtained from the gyroscope
     */
    public void gyro(double angle) {
        currentAngle = angle;
    }

    /**
     * Resets zero to current position
     */
    public void resetZero() {
        zero = currentAngle;
    }

    /**
     * Turns a specified amount
     *
     * @param delta The amount to turn
     */
    public void turn(double delta) {
        setAngle = wrap(currentAngle - delta);
        turning = true;
    }

    /**
     * Does the actual processing and driving
     * Call once per loop
     */
    public void driive() {
        // Accounts for rotation using gyro values (if field centric)
        if(fieldCentric) {
            theta += (currentAngle + zero);
        }

        // Code for automated turning
        if(righteous || turning) {
            // Sets amount to turn based on target position
            if(turn == 0.0) {
                turn = wrap(currentAngle - setAngle) / 18;
            }
            // Sets target to current if user is manually turning
            else {
                setAngle = currentAngle;
            }

            // Avoids low power/squeaking motors
            if(Math.abs(turn) < 0.05) turn = 0;
            if(turn < 0.15 && turn > 0.05) turn = 0.15;
            if(turn > -0.15 && turn < -0.05) turn = -0.15;
        }

        // Deactivate turning when the turn is done
        if(turning && turn == 0) {
            turning = false;
        }

        // Applies the speed setting
        r *= speed / 10.0;
        turn *= speed / 10.0;

        // Send power to wheels
        for(int i = 0; i != wheels.length; i++) {
            wheels[i].setPower(Math.sin(wheelAngles[i] - theta) * r + turn);
            wheelPowers[i] = Math.sin(wheelAngles[i] - theta) * r + turn;
        }
    }

    public void updateTelemetry(TelemetryPacket packet) {
        packet.put("currentAngle", currentAngle);
        packet.put("zero", zero);
        packet.put("theta", theta);
        packet.put("r", r);
        packet.put("setAngle", setAngle);
        packet.put("turn", turn);
        packet.put("turning", turning);
        for(int i = 0; i != wheelPowers.length; i++) {
            packet.put("wheel power " + i, wheelPowers[i]);
        }
    }

    private double wrap(double input) {
        while(Math.abs(input) > 180) {
            if(input < -180) {
                input += 360;
            }
            else {
                input -= 360;
            }
        }
        return input;
    }

}