package com.tigerhuang.motion_profile2;

/**
 * Contains several utility methods for converting waypoints into profiles
 *
 * @author Tiger Huang
 */
public class SplineGenerator {
	/**
	 * Creates a list of QuinticBeziers from a list of waypoints
	 * There must be at least 2 waypoints
	 *
	 * @param waypoints an array of waypoints to use
	 * @return an array of QuinticBeziers that satisfies the waypoints
	 */
	public static QuinticBezier[] splinesFromWaypoints(Waypoint[] waypoints) {
		QuinticBezier[] splines = new QuinticBezier[waypoints.length - 1];
		for(int i = 0; i < splines.length; i++) {
			Waypoint wp1 = waypoints[i];
			Waypoint wp2 = waypoints[i+1];
			splines[i] = new QuinticBezier(wp1, wp2);
		}
		return splines;
	}

	/**
	 * Performs arc length parameterization
	 * 
	 * @param sampleLength The target arc length of the output segments
	 * @return An array of arrays. The list of arrays is given below.<br>
	 * 0. Times (length n rounded up to nearest multiple of 1024)<br>
	 * 1. Distance between times (length n-1)<br>
	 * 3. Velocities at times [uninitialized] (length n)
	 */
	public static double[][] uniformLengthSegmentData(QuinticBezier[] splines, double sampleLength) {
		double t = 0;                 // Time
		int i = 0;                    // Current index
		int ci = 1024;                // Capacity increment
		double[] ts = new double[ci]; // Accumulator
		for(;;) {
			// Add current time
			ts[i++] = t;
			if(i >= ts.length) {
				double[] tso = ts;
				ts = new double[tso.length + ci];
				System.arraycopy(tso, 0, ts, 0, tso.length);
			}

			// Prepare for next step
			double rt = t%1;                                  // Relative time
			QuinticBezier s = splines[(int)t];                // Current spline
			double dl = Math.hypot(s.getDX(rt), s.getDY(rt)); // Length per time
			double dt = sampleLength/dl;                      // Time change

			// Check for end condition
			if(t+dt*2 >= splines.length) {
				// Add additional point as long as the last segment will be more than half the target length
				if((1-rt)*dl > sampleLength*3/2) {
					ts[i++] = t+dt;
					if(i >= ts.length) {
						double[] tso = ts;
						ts = new double[tso.length + ci];
						System.arraycopy(tso, 0, ts, 0, tso.length);
					}
				}
				// Add final point
				ts[i++] = Math.nextDown(splines.length);
				break;
			}

			// Normal condition
			else {
				t += dt;
			}
		}

		// Generate distances
		double[] ls = new double[i-1];
		double x0 = splines[0].getX(0);
		double y0 = splines[0].getY(0);
		for(int j = 0; j < ls.length; j++) {
			double t1 = ts[j+1];
			QuinticBezier s1 = splines[(int)t1];
			double x1 = s1.getX(t1%1);
			double y1 = s1.getY(t1%1);
			ls[j] = Math.hypot(x1-x0, y1-y0);
			x0 = x1;
			y0 = y1;
		}

		// Generate velocity
		double[] vs = new double[i];

		// Export arrays
		double[][] result = new double[3][];
		result[0] = ts;
		result[1] = ls;
		result[2] = vs;
		return result;
	}

	/**
	 * Generates a velocity profile with the given data.
	 * 
	 * @param splineData The spline data produced by uniformLengthSegmentData
	 * @param splines The array of splines being used
	 * @param limits The array of limits to be applied
	 * @param wheelbase The distance between the robot's wheels
	 * @param max_iterations The maximum number of runs allowed for the iterative solver
	 * @param adjust_scale The scale factor for the adjustment when voltage limiting
	 * @param adjust_offset The offset for the adjustment when voltage limiting
	 * @param smoothing The smoothing done between adjustment rounds
	 * @param v_initial The initial maximum velocity (not accurate)
	 * @param v_final The final maximum velocity (not accurate)
	 * @param v_max The maximum linear velocity
	 * @param v_tau The velocity time constant for linear motion at max voltage
	 * @param w_max The maximum angular velocity
	 * @param w_tau The velocity time constant for angular motion at max voltage
	 * @return None
	 */
	public static void generateVelocityProfile(double[][] splineData,
			QuinticBezier[] splines, Limit[] limits,
			double wheelbase, double smoothing,
			int max_iterations, double adjust_scale, double adjust_offset,
			double v_initial, double v_final,
			double v_max, double v_tau,
			double w_max, double w_tau) {

		// Explode array
		int l = splineData[2].length;
		double[] ts = splineData[0];
		double[] ls = splineData[1];
		double[] vs = splineData[2];
		double[] vsn = new double[l];

		// Generate curvatures
		double[] ks = new double[l];
		for(int j = 0; j < ks.length; j++) {
			double t = ts[j];
			ks[j] = splines[(int)t].getCurvature(t%1);
		}

		// Sweep forward and backwards until a cycle does not find any more points to adjust
		boolean first = true;
		boolean even = true;
		boolean changed = true;
		for(int z = 0; z < max_iterations; z++) {
			// Handle loop exit
			if(even) {
				if(!changed) {
					System.out.println("Exited early at iteration: " + z);
					break;
				}
				changed = false;
			}

			// Handle simultaneous update
			System.arraycopy(vs, 0, vsn, 0, l);

			// Global maximum velocity
			for(int i = 0; i < l; i++) {
				double v = Math.min(v_max/(1+Math.abs(ks[i])*wheelbase/2), w_max/Math.abs(ks[i]));
				if(first || v < vsn[i]) {
					vsn[i] = v;
					changed = true;
				}
			}
			vsn[0] = Math.min(vsn[0], v_initial);
			vsn[l-1] = Math.min(vsn[l-1], v_final);
			first = false;

			// Velocity limits
			for(Limit limit:limits) {
				if(limit.type == LimitType.VELOCITY) {
					for(int i = 0; i < l; i++) {
						if(ts[i] < limit.t1) {
							continue;
						}
						if(ts[i] > limit.t2) {
							break;
						}
						if(vsn[i] > limit.limit) {
							vsn[i] = limit.limit;
							changed = true;
						}
					}
				}
			}

			// Processing loop
			int i = even ? 0 : l-2;
			for(;;) {
				// Check acceleration limits
				double lowest = 2*v_max/v_tau;
				for(Limit limit:limits) {
					if(ts[i] >= limit.t1 && ts[i+1] <= limit.t2) {
						if(limit.type == LimitType.ACCELERATION) {
							if(limit.limit < lowest) {
								lowest = limit.limit;
							}
						}
					}
				}
				double vmin = Math.min(vsn[i], vsn[i+1]);
				double vmax = Math.sqrt(vmin*vmin + 2*lowest*ls[i]);
				if(vmax < vsn[i]) {
					vsn[i] = vmax;
					changed = true;
				}
				if(vmax < vsn[i+1]) {
					vsn[i+1] = vmax;
					changed = true;
				}

				// Check voltage limits
				lowest = Double.MAX_VALUE;
				for(Limit limit:limits) {
					if(ts[i] >= limit.t1 && ts[i+1] <= limit.t2) {
						if(limit.type == LimitType.VOLTAGE) {
							if(limit.limit < lowest) {
								lowest = limit.limit;
							}
						}
					}
				}
				if(lowest < Double.MAX_VALUE) {
					double A = v_tau/(2*ls[i]*v_max);
					double B = w_tau*(ks[i]+ks[i+1])/(4*ls[i]*w_max);
					double C = w_tau*(ks[i+1]-ks[i])/(4*ls[i]*w_max);
					double D = 1/(2*v_max);
					double E = wheelbase*(ks[i]+ks[i+1])/(8*v_max);
					double v1 = vsn[i];
					double v2 = vsn[i+1];
					double vl = (A-B)*(v2*v2-v1*v1) - C*(v1+v2)*(v1+v2) + (D-E)*(v1+v2);
					double vr = (A+B)*(v2*v2-v1*v1) + C*(v1+v2)*(v1+v2) + (D+E)*(v1+v2);
					if(Math.abs(vl) > lowest) {
						vsn[i] -= vsn[i] / v_max * adjust_scale * (Math.abs(vl)/lowest-adjust_offset);
						vsn[i+1] -= vsn[i+1] / v_max * adjust_scale * (Math.abs(vl)/lowest-adjust_offset);
						changed = true;
					}
					if(Math.abs(vr) > lowest) {
						vsn[i] -= vsn[i] / v_max * adjust_scale * (Math.abs(vr)/lowest-adjust_offset);
						vsn[i+1] -= vsn[i+1] / v_max * adjust_scale * (Math.abs(vr)/lowest-adjust_offset);
						changed = true;
					}
				}

				// Loop handling
				if(even) {
					i++;
					if(i >= l-1) {
						break;
					}
				}
				else {
					i--;
					if(i < 0) {
						break;
					}
				}
			}

			// Handle simultaneous update
			vs[0] = vsn[0];
			vs[l-1] = vsn[l-1];
			for(i = 1; i < l-1; i++) {
				vs[i] = smoothing/2*vsn[i-1]+(1-smoothing)*vsn[i]+smoothing/2*vsn[i+1];
			}
			for(i = 0; i < l; i++) {
				if(vs[i] < 0) {
					vs[i] = 1;
				}
			}
			even = !even;
		}
	}

	/**
	 * Generates a time based profile with the given data.
	 * 
	 * @param splineData The spline data produced by generateVelocityProfile
	 * @param splines The array of splines being used
	 * @param dt The timestep
	 * @param wheelbase The distance between the robot's wheels
	 * @param filter_length The length of the boxcar filter applied at the end in number of timesteps
	 * @param v_max The maximum linear velocity
	 * @param v_tau The velocity time constant for linear motion at max voltage
	 * @param w_max The maximum angular velocity
	 * @param w_tau The velocity time constant for angular motion at max voltage
	 * @return An array of arrays. The list of arrays is given below.<br>
	 * 0. Linear position<br>
	 * 1. Linear velocity<br>
	 * 2. Linear acceleration<br>
	 * 3. Angular position<br>
	 * 4. Angular velocity<br>
	 * 5. Angular acceleration<br>
	 * 6. Left voltage<br>
	 * 7. Right voltage
	 */
	public static double[][] generateTimeParameterizedProfile(double[][] splineData,
			QuinticBezier[] splines,
			double dt, double wheelbase, int filter_length,
			double v_max, double v_tau,
			double w_max, double w_tau) {

		// Explode array
		int l = splineData[2].length;
		double[] us = splineData[0];
		double[] ls = splineData[1];
		double[] vs = splineData[2];

		// Find times
		double[] ts = new double[l];
		ts[0] = 0;
		for(int i = 0; i < l-1; i++) {
			ts[i+1] = ts[i] + 2*ls[i] / (vs[i]+vs[i+1]);
		}

		// Initialize arrays
		int c = (int)Math.ceil(ts[l-1]/dt);
		double[] lp = new double[c];
		double[] ap = new double[c];
		double[] vl = new double[c];
		double[] vr = new double[c];
		double[] uts = new double[c];
		double[] lv = new double[c];
		double[] av = new double[c];
		double[] ks = new double[c];
		double[] la = new double[c];
		double[] aa = new double[c];

		// Find time parameterized u, linear position, velocity
		uts[0] = 0;
		lv[0] = vs[0];
		lp[0] = 0;
		lp[1] = 0;
		double l0 = 0;
		for(int i = 0, j = 1; i < l-1 && j < c;) {
			double t = dt*j;
			if(ts[i+1] >= t) {
				double x = (t-ts[i]) / (ts[i+1]-ts[i]);
				uts[j] = (1-x)*us[i] + x*us[i+1];
				lv[j] = (1-x)*vs[i] + x*vs[i+1];
				lp[j] = l0 + x*ls[i];
				j++;
			}
			else {
				l0 += ls[i];
				i++;
			}
		}

		// Find heading
		double h0 = splines[0].getHeading(0);
		ap[0] = 0;
		for(int j = 1; j < c; j++) {
			double u = uts[j];
			ap[j] = splines[(int)u].getHeading(u%1) - h0;
			if(ap[j]-ap[j-1] > Math.PI) {
				ap[j] -= Math.PI*2;
			}
			if(ap[j]-ap[j-1] < -Math.PI) {
				ap[j] += Math.PI*2;
			}
		}

		// Find curvature and angular velocity
		for(int j = 0; j < c; j++) {
			double u = uts[j];
			ks[j] = splines[(int)u].getCurvature(u%1);
			av[j] = lv[j] * ks[j];
		}

		// Find voltages
		// http://dynref.engr.illinois.edu/rkt.html
		for(int j = 0; j < c-1; j++) {
			double dl = lp[j+1] - lp[j];
			double u1 = uts[j];
			double k1 = splines[(int)u1].getCurvature(u1%1);
			double v1 = lv[j];
			double u2 = uts[j+1];
			double k2 = splines[(int)u2].getCurvature(u2%1);
			double v2 = lv[j+1];
			double A = v_tau/(2*dl*v_max);
			double B = w_tau*(k1+k2)/(4*dl*w_max);
			double C = w_tau*(k2-k1)/(4*dl*w_max);
			double D = 1/(2*v_max);
			double E = wheelbase*(k1+k2)/(8*v_max);
			vl[j] = (A-B)*(v2*v2-v1*v1) - C*(v1+v2)*(v1+v2) + (D-E)*(v1+v2);
			vr[j] = (A+B)*(v2*v2-v1*v1) + C*(v1+v2)*(v1+v2) + (D+E)*(v1+v2);
		}
		vl[c-1] = vl[c-2];
		vr[c-1] = vr[c-2];

		// Find linear and angular acceleration
		for(int j = 0; j < c-1; j++) {
			la[j] = (lv[j+1]-lv[j]) / dt;
			aa[j] = (av[j+1]-av[j]) / dt;
		}
		la[c-1] = la[c-2];
		aa[c-1] = aa[c-2];

		// Apply boxcar filter
		double[][] unfiltered = {lp, lv, la, ap, av, aa, vl, vr};
		double[][] result = new double[8][c+filter_length];
		for(int i = 0; i < 8; i++) {
			for(int j = 0; j < c+filter_length; j++) {
				result[i][j] = 0;
				for(int k = j-filter_length; k <= j; k++) {
					int index = k;
					if(index < 0) {
						index = 0;
					}
					if(index > c-1) {
						index = c-1;
					}
					result[i][j] += unfiltered[i][index];
				}
				result[i][j] /= filter_length+1;
			}
		}
		return result;
	}
}
