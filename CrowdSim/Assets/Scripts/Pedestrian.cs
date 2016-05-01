using UnityEngine;
using System.Collections;


public class Pedestrian : Obstacle {

    public const float EULER = 2.71828f;

    public Pedestrian(Transform goal, Vector3 vComf, float fov, float visRange, float a, float b, float c, float tau2, float rayCastGranularity)
        : base(goal, vComf, fov, visRange, a, b, c, tau2, rayCastGranularity) {
       
    }

    protected override float computeAngularVelocity() {
        float angVel = getThetaDerivative();
        return angVel;
    }

    protected override Vector2 computeTangentialVelocity() {
        float ttiMP = getTTIMP();
        float exp = -0.5f * Mathf.Pow(ttiMP, 2.0f);
 
        if (getPPos().Count == 0) {
            return vComf;
        }else{
            return vComf * (1 - Mathf.Pow(EULER, exp));  
        }
    }
}
