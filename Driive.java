package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

class Driive {
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
    boolean fieldCentric = true;

    /**
     * Marks whether the robot fights to maintain its orientation
     * Defaults to false
     */
    boolean righteous = false;

    /**
     * Speed, works on a scale of 1-10
     * Affects the overall speed of the robot
     */
    int speed = 10;

    /**
     * Initializes the Driive instance
     * Called once in init
     *
     * @param wheels An array of the DcMotors to be used for driving
     * @param wheelAngles An array of the physical angles of the wheels, e.g. 0,90,180,270
     * @throws java.lang.ArrayIndexOutOfBoundsException - if wheels[] and wheelAngles[] are different lengths
     */
    void init(DcMotor[] wheels, double[] wheelAngles) throws Exception{
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
    void init(DcMotor[] wheels, double[] wheelAngles, double zero) throws Exception{
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
    void cartesian(double x, double y, double turn) {
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
    void polar(double r, double theta, double turn) {
        this.r = r;
        this.theta = theta;
        this.turn = turn;
    }

    /**
     * Drives in a given direction at a given speed for a given distance, using encoders.
     * Blocking function, will not exit until drive is completed.
     *
     * @param r Speed
     * @param theta Direction in radians from zero
     * @param distance Distance in ____
     */
    void polarAuto(double r, double theta, double distance, TeleAuto callback) {
        this.r = r;
        this.theta = theta;

        // Records previous preferences
        boolean righteousPrev = righteous;
        boolean fieldCentricPrev = fieldCentric;
        fieldCentric = true;
        righteous = true;

        // Gets initial wheel positions
        double[] wheelDistances = new double[wheels.length];
        for(int i = 0; i < wheels.length; i++) {
            wheelDistances[i] = wheels[i].getCurrentPosition();
        }

        while(callback.opModeIsActive()) {
            // Drive
            driive();
            TelemetryPacket packet = new TelemetryPacket();

            // Find the current total average delta
            double total = 0;
            for(int i = 0; i < wheels.length; i++) {
                packet.put("wheel position " + i, wheels[i].getCurrentPosition());
                double currentWheelSin = Math.sin(wheelAngles[i] - theta);
                double wheelDistance = wheels[i].getCurrentPosition() - wheelDistances[i];
                double wheelRobotDistance = currentWheelSin * wheelDistance;
                total += wheelRobotDistance;
            }
            total /= wheels.length;

            // Add data to telemetry
            updateTelemetry(packet);
            packet.put("total", total);
            callback.updateAuto(packet);

            // Stop driving
            if (Math.abs(total) > Math.abs(distance) && turn == 0) break;
        }

        // Stop driving
        this.r = 0;
        driive();

        // Resets to previous values
        fieldCentric = fieldCentricPrev;
        righteous = righteousPrev;
    }

    /**
     * Sets the gyroscope value
     * Call every loop with updated values
     *
     * @param angle The angle obtained from the gyroscope
     */
    void gyro(double angle) {
        currentAngle = angle;
    }

    /**
     * Resets zero to current position
     */
    void resetZero() {
        zero = -currentAngle;
    }

    /**
     * Turns a specified amount
     *
     * @param delta The amount to turn
     */
    void turnRel(double delta) {
        setAngle = wrap(currentAngle - delta);
        turning = true;
    }

    /**
     * Turns to a specified angle
     *
     * @param angle The angle to turn to
     */
    void turnAbs(double angle) {
        setAngle = angle;
        turning = true;
    }

    /**
     * Does the actual processing and driving
     * Call once per loop
     */
    void driive() {
        // Accounts for rotation using gyro values (if field centric)
        double localtheta = theta;
        if(fieldCentric) {
            localtheta += (currentAngle + zero);
        }

        // Code for automated turning
        if(righteous || turning) {
            // Sets amount to turn based on target position
            if(turn == 0.0) {
                turn = Math.pow(wrap(currentAngle - setAngle), 3);
            }
            // Sets target to current if user is manually turning
            else {
                setAngle = currentAngle;
            }

            double min = 0.05;
            double max = 0.10;

            // Avoids low power/squeaking motors
            if(Math.abs(turn) < min) turn = 0;
            if(turn < max && turn > min) turn = max;
            if(turn > -max && turn < -min) turn = -max;
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
            wheels[i].setPower(Math.sin(wheelAngles[i] - localtheta) * r + turn);
            wheelPowers[i] = Math.sin(wheelAngles[i] - localtheta) * r + turn;       // Telemetry
        }
    }

    /**
     * Updates the telemetry packet with driving info
     *
     * @param packet Packet to add data to
     */
    void updateTelemetry(TelemetryPacket packet) {
        packet.put("currentAngle", currentAngle);
        packet.put("zero", zero);
        packet.put("theta", theta);
        packet.put("r", r);
        packet.put("setAngle", setAngle);
        packet.put("turn", turn);
        packet.put("turning", turning);
        packet.put("fieldCentric", fieldCentric);
        packet.put("righteous", righteous);
        for(int i = 0; i != wheelPowers.length; i++) {
            packet.put("wheel power " + i, wheelPowers[i]);
        }
    }

    private double wrap(double input) {
        double pi = Math.PI;
        while(Math.abs(input) > pi) {
            if(input < -pi) {
                input += 2*pi;
            }
            else {
                input -= 2*pi;
            }
        }
        return input;
    }
}

interface TeleAuto {
    void updateAuto(TelemetryPacket packet);
    boolean opModeIsActive();
}