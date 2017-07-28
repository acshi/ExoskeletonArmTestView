using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ExoskeletonArmTestView {
    static class PathPlanner {
        // dimensions in inches...
        // origin is defined at the sholder where the upper arm extends out
        // y is the vertical dimension. z is the forward dimension.

        // lower arm length
        const float l1 = 12.0625f;
        // upper arm length
        const float l2 = 9.5625f;

        static double rad(double a) {
            return a / 180.0 * Math.PI;
        }

        static double deg(double a) {
            return a * 180.0 / Math.PI;
        }

        static float sin(float a) {
            return (float)Math.Sin(rad(a));
        }

        static float cos(float a) {
            return (float)Math.Cos(rad(a));
        }

        static float acos(float a) {
            return (float)deg(Math.Acos(a));
        }

        static float asin(float a) {
            return (float)deg(Math.Asin(a));
        }

        static float atan2(float y, float x) {
            return (float)deg(Math.Atan2(y, x));
        }

        // Order is sholder, upper arm, lower arm
        // in a vector3 they may be referenced as "x", "y", and "z"
        static readonly float[] minAngles = new float[] { 10f, -10f, 80f };
        static readonly float[] maxAngles = new float[] { 170f, 170f, 167.5f };

        // law of cosines...
        static readonly float minSphereRadius = (float)Math.Sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(minAngles[2]));
        static readonly float maxSphereRadius = (float)Math.Sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(maxAngles[2]));

        const float safetyK = 1.2f;

        // scale positions within the minimum sphere or outside the maximum sphere
        // so that they are possible to reach.
        // Returns an arbitrary point if p is (0, 0, 0)
        static Vector3 constrainXyz(Vector3 p) {
            float r = p.Mag();
            if (r == 0) {
                return new Vector3(0, 0, safetyK * minSphereRadius);
            }
            if (r < minSphereRadius) {
                float factor = (safetyK * minSphereRadius) / r;
                return p.Mul(factor);
            } else if (r > maxSphereRadius) {
                float factor = (maxSphereRadius / safetyK) / r;
                return p.Mul(factor);
            }
            return p;
        }

        public static Vector3 AnglesToXyz(Vector3 angles) {
            float Q = l2 * cos(angles[2]) - l1;
            float M = l2 * sin(angles[2]);
            float y = Q * cos(angles[1]) + M * sin(angles[1]);
            float r = Q * sin(angles[1]) - M * cos(angles[1]);
            // r is the radial distance in the x-z plane from the sholder
            // we can decompose it using the sholder angle to get x and z
            float z = r * sin(angles[0]);
            float x = r * -cos(angles[0]);

            return new Vector3(x, y, z);
        }

        static float safeSqrt(float f) {
            if (f < 0 && f > -1e-3) {
                return 0;
            }
            return (float)Math.Sqrt(f);
        }

        static Vector3 xyzToAngles(Vector3 xyz) {
            float omega = atan2(xyz.z, xyz.x);
            float mag = xyz.Mag();
            float theta = acos((l2 * l2 + l1 * l1 - mag * mag) / (2 * l1 * l2));
            float Q = l2 * cos(theta) - l1;
            float M = l2 * sin(theta);
            float phi = 2f * atan2(M - safeSqrt(M * M + Q * Q - xyz.y * xyz.y), Q + xyz.y);
            if (phi < 0) {
                phi += 360;
            }
            //float phi2 = 2f * atan2(M + (float)Math.Sqrt(M * M + Q * Q - xyz.y * xyz.y), Q + xyz.y);
            if (float.IsNaN(omega) || float.IsNaN(phi) || float.IsNaN(theta)) {
                omega = 0;
            }
            return new Vector3(-omega, phi, theta);
        }

        static Vector3[] solveForPath(Vector3 currentAngles, Vector3 targetXyz, float stepDelta) {
            Vector3 currentXyz = AnglesToXyz(currentAngles);
            Vector3 backToAngles = xyzToAngles(currentXyz);
            targetXyz = constrainXyz(targetXyz);

            Vector3 trajectory = targetXyz.Sub(currentXyz);
            float distance = trajectory.Mag();
            Vector3 pathStep = trajectory.Mul(stepDelta / distance);

            int numSteps = (int)Math.Ceiling(distance / stepDelta);
            Vector3[] pathPoints = new Vector3[numSteps];
            float maxDeviance = 0;
            int maxDevianceI = -1;
            for (int i = 0; i < numSteps - 1; i++) {
                Vector3 idealPoint = currentXyz.Add(pathStep.Mul(i + 1));
                Vector3 constrainedPoint = constrainXyz(idealPoint);
                float deviance = idealPoint.Sub(constrainedPoint).Mag();
                if (deviance > maxDeviance) {
                    maxDeviance = deviance;
                    maxDevianceI = i;
                }
                pathPoints[i] = constrainedPoint;
            }
            pathPoints[numSteps - 1] = targetXyz;

            if (maxDevianceI != -1) {
                return solveForPath(currentAngles, pathPoints[maxDevianceI], stepDelta);
            }

            return pathPoints;
        }

        public static Vector3 SolveForNextAnglesInPath(Vector3 currentAngles, Vector3 targetXyz, float stepDelta) {
            for (int i = 0; i < 3; i++) {
                if (currentAngles[i] < minAngles[i] || currentAngles[i] > maxAngles[i]) {
                    throw new ArgumentOutOfRangeException("currentAngles is not a possible configuration");
                }
            }

            Vector3[] pathPoints = solveForPath(currentAngles, targetXyz, stepDelta);
            Vector3 nextPoint = pathPoints.Length > 1 ? pathPoints[1] : pathPoints[0];
            return xyzToAngles(nextPoint);
        }
    }
}
