package frc.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedMap;
import java.util.TreeMap;

public class InterCalculator {
    private SortedMap<Double, InterParameter> parameterMap = new TreeMap<>();

    public InterCalculator(InterParameter... parameters) {
        for(InterParameter p : parameters) {
            parameterMap.put(p.x, p);
        }
    }
    
    /**
     * Calculates an interpolated InterParameter based on value x
     * 
     * If x is less/greater than the least/greatest value of x recorded in the interpolation table, the first/last InterParameter
     * will be returned regardless of the magnitude of x.
     * 
     * @param x x value to interpolate for
     * @return Interpolated parameter InterParameter
     */
    public InterParameter calculateParameter(double x) {
        // TreeMap should maintain sorted state
        List<InterParameter> sortedParameters = new ArrayList<>(parameterMap.values());

        if(parameterMap.get(x) != null) {
            return parameterMap.get(x);
        }

        // alternate behavior: take last two/first two points and extrapolate unknown area
        if(x < sortedParameters.get(0).x) {
            return sortedParameters.get(0);
        } else if(x > sortedParameters.get(sortedParameters.size() - 1).x) {
            return sortedParameters.get(sortedParameters.size() - 1);
        }

        InterParameter lower = null;
        InterParameter upper = null;

        for(int i = 1; i < sortedParameters.size(); i++) {
            if(sortedParameters.get(i).x >= x) {
                lower = sortedParameters.get(i - 1);
                upper = sortedParameters.get(i);
                break;
            }
        }

        if(lower == null || upper == null) {
            // this should never happen by virtue of the precondition "x > sortedParameters.get(sortedParameters.size() - 1).x".
            // However, I placed it here so our robot isn't brought to it's knees because of an interpolation issue - just in case.
            return sortedParameters.get(sortedParameters.size() - 1);
        }

        return lower.interpolate(upper, x - lower.x);
    }
}