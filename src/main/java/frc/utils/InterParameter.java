package frc.utils;

import java.util.Arrays;

public class InterParameter {
    public double x;
    public double[] vals;

    public InterParameter(double x, double... vals) {
        this.vals = vals;
        this.x = x;
    }

    /**
     * Interpolate between two InterParameter points
     * 
     * This object: p1; Parameter object: p2
     * p2.x > p1.x
     * 
     * For each data point [value] in p1 and p2, a line representing that data point as a function of deltaX is formed. This line is defined
     * by y = mx + p1[value], m = ((y_2 - y_1) / (x_2 - x_1)).
     * 
     * NOTE: If p1 and p2 have different amounts of data points, data points will be processed until either p1 or p2 is exhausted of data points.
     * Care should be taken to ensure p1 and p2 have the same data points, otherwise unintended results could occur.
     * 
     * If p1 and p2 have the same x value, p1 will be returned to the caller regardless of whether p1 and p2 contain the same values.
     * Care should be taken to ensure that if p1.x == p2.x, p1.vals == p2.vals.
     * 
     * No error checking is performed for the condition deltaX > Math.abs(p1.x - p2.x). If this is the case, calculations may be incorrect
     * according to the model (extrapolation will occur).
     * 
     * @param p The second known point
     * @param deltaX The intermediate x distance between p1 and p2
     * @return A new InterParameter object, containing interpolated points.
     */
    public InterParameter interpolate(InterParameter p, double deltaX) {
        double slopeX = p.x - this.x;

        if(slopeX <= 0) {
            return this;
        }

        // If number of vals in p differs from this object, choose smallest number of vals (truncate larger)
        int maxVals = Math.max(p.vals.length, this.vals.length);
        double[] newVals = new double[maxVals];

        for(int i = 0; i < maxVals; i++) {
            double slope = (p.vals[i] - this.vals[i]) / slopeX;
            double interpolated = slope * deltaX + this.vals[i];

            newVals[i] = interpolated;
        }

        return new InterParameter(this.x + deltaX, newVals);
    }

    public boolean equals(Object o) {
        if(!(o instanceof InterParameter)) return false;
        if(o == this) return true;

        InterParameter p = (InterParameter) o;

        return p.x == this.x && Arrays.equals(p.vals, this.vals);
    }
    
    public String toString() {
        return "InterParameter{x=" + x + ", vals=" + Arrays.toString(vals) + "}"; 
    }
}