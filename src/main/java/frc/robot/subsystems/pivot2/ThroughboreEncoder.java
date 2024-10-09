package frc.robot.subsystems.pivot2;

import com.revrobotics.SparkAbsoluteEncoder;

public class ThroughboreEncoder {
    SparkAbsoluteEncoder abs;
    double offset;

    public ThroughboreEncoder(SparkAbsoluteEncoder absoluteEncoder, double absOffset) {
        abs = absoluteEncoder;
        offset = absOffset;
    }

    public double getPosition() {
        return (abs.getPosition() + offset) % (2 * Math.PI);
    }

    public void setOffset(double absOffset) {    
        offset = absOffset;
    }

    public double getOffset() {
        return offset;
    }
}
