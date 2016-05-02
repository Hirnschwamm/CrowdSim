using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public abstract class Obstacle : MonoBehaviour {
    public Rigidbody2D body;
    public CircleCollider2D circleCollider; 
    public Transform goal; //The target this obstacle tries to reach 
    public Vector3 vComf; //Comfort velocity 
    public List<Rigidbody2D> p; //Points near the obstacle
    public List<Rigidbody2D> pCol; //Points that the obstacle is in risk of colliding with and that need to be reacted to 
    public float fov; //field of view of this obstacle in degrees
    public float visRange; //visual range
    public float a; //constant needed to compute tau
    public float b; //constant needed to compute tau
    public float c; //constant needed to compute tau
    public float tau2; //constant needed to regulate tangential velocity
    public float rayCastGranularity; //Granularity in degrees with which to cast the rays for obstacle detection
    public bool drawGizmos;
   

    public Obstacle(Transform goal, Vector3 vComf, float fov, float visRange, float a, float b, float c, float tau2, float rayCastGranularity) {
        this.goal = goal;
        this.vComf = vComf;
        this.fov = fov;
        this.visRange = visRange;
        this.a = a;
        this.b = b;
        this.c = c;
        this.tau2 = tau2;
        this.rayCastGranularity = rayCastGranularity;
    }

    // Use this for initialization
    void Start(){
        p = new List<Rigidbody2D>();
        pCol = new List<Rigidbody2D>();
    }

    // Update is called once per frame
    void Update(){
        p = getP();
        float distanceThreshold = 0.2f;

        if (Vector2.Distance(transform.position, goal.position) > distanceThreshold) {
            vComf = transform.up;
            body.angularVelocity = computeAngularVelocity();
            body.velocity = computeTangentialVelocity();
        } else {
            body.velocity = new Vector2(0.0f, 0.0f);
            body.angularVelocity = 0.0f;
        }
    }

    protected abstract float computeAngularVelocity();

    protected abstract Vector2 computeTangentialVelocity();

    //Function to collect all points of vision for a walker. In the paper those were collected via synthetic vision
    protected List<Rigidbody2D> getP() {
        List<Rigidbody2D> p = new List<Rigidbody2D>();
        Vector2 orientation = transform.up;
        float colliderBuffer = circleCollider.radius - 0.1f;
        float fovHalf = fov / 2;
        for (float i = -fovHalf; i < fovHalf; i += rayCastGranularity) {
            Vector3 rayDir = rotateVector2(orientation, i);
            RaycastHit2D[] hits = Physics2D.RaycastAll(transform.position + rayDir.normalized * colliderBuffer, rayDir.normalized, visRange);
            foreach(RaycastHit2D hit in hits){
                if (hit && !p.Contains(hit.rigidbody)) {
                    p.Add(hit.rigidbody);
                }
            }
        }
        return p;
    }

    void OnDrawGizmosSelected() {
        if (drawGizmos) {
            Vector2 orientation = transform.up;
            float colliderBuffer = circleCollider.radius - 0.1f;
            float fovHalf = fov / 2;
            for (float i = -fovHalf; i < fovHalf; i += rayCastGranularity) {
                Vector3 rayDir = rotateVector2(orientation, i);
                Gizmos.DrawLine(transform.position + rayDir.normalized * colliderBuffer, transform.position + rayDir.normalized * visRange);
            }
        }
    }

    void OnDrawGizmos() {
        Gizmos.DrawLine(transform.position, body.position + body.velocity);
    }

    private Vector2 rotateVector2(Vector2 v, float deg) {
        float rad = deg * Mathf.Deg2Rad;
        float sin = Mathf.Sin(rad);
        float cos = Mathf.Cos(rad);

        return new Vector2((cos * v.x - sin * v.y), (sin * v.x + cos * v.y)); 
    }
 
    protected float getAlpha(Rigidbody2D other){
        Vector2 v = body.velocity.normalized;
        Vector2 k = (other.position - body.position).normalized;
        int sign = Vector3.Cross(new Vector3(v.x, v.y), new Vector3(k.x, k.y)).z < 0 ? -1 : 1;
        float alpha = Vector2.Angle(v, k);
        return alpha * sign;
    }

    protected float getAlpha(Transform other) {
        Vector2 v = body.velocity.normalized;
        Vector2 k = (new Vector2(other.position.x, other.position.y) - body.position).normalized;
        int sign = Vector3.Cross(new Vector3(v.x, v.y), new Vector3(k.x, k.y)).z < 0 ? -1 : 1;
        float alpha = Vector2.Angle(v, k);
        return alpha * sign;
    }

    protected float getAlphaDerivative(Rigidbody2D other) {
        Vector2 v = body.velocity;
        Vector2 k = (body.position - other.position).normalized;
        float d = Vector2.Distance(body.position, other.position);
        Vector2 vO = other.velocity;
        Vector2 vRel = vO - v;
        Vector2 vConv = Vector2.Dot(vRel, k) * k;
        Vector2 vOrth = vRel - vConv;
        float alphaDerivative = Mathf.Atan(vOrth.magnitude / (d - vConv.magnitude)) * Time.deltaTime; //Paper deviation: "* Time.deltaTime" (integration) instead of "* (Time.deltaTime ^ -1)"(derivative) 
        return alphaDerivative;
    }

    protected float getAlphaDerivative(Transform other) {
        Vector2 v = body.velocity;
        Vector2 k = (body.position - new Vector2(other.position.x, other.position.y)).normalized;
        float d = Vector2.Distance(body.position, other.position);
        Vector2 vO = new Vector2(0.0f, 0.0f);
        Vector2 vRel = vO - v;
        Vector2 vConv = Vector2.Dot(vRel, k) * k;
        Vector2 vOrth = vRel - vConv;
        float alphaDerivative = Mathf.Atan(vOrth.magnitude / (d - vConv.magnitude)) * Time.deltaTime; //Paper deviation: "* Time.deltaTime" (integration) instead of "* Time.deltaTime^-1"(derivative)
        
        return alphaDerivative;
    }

    protected float getTTI(Rigidbody2D other) {
        Vector2 v = body.velocity;
        Vector2 k = (body.position - other.position).normalized; 
        float d = Vector2.Distance(body.position, other.position);
        Vector2 vO = other.velocity;
        Vector2 vRel = vO - v;
        Vector2 vConv = Vector2.Dot(vRel, k) * k;
        float tti = d * Mathf.Pow(vConv.magnitude, -1.0f);
        return tti;
    }

    protected float getTTIMP() {
        float min = Mathf.Infinity;
        foreach (Rigidbody2D point in getPPos()) {
            float tti = getTTI(point);
            if (tti > 0.0f && tti < min) {
                min = tti;
            }
        }
        return min;
    }

    protected float getTau1(float tti, Rigidbody2D other) {
        float alphaDerivative = getAlphaDerivative(other);
        if (alphaDerivative < 0.0f) {
            return getTau1Minus(tti);
        } else {
            return getTau1Plus(tti);
        }
    }

    protected float getTau1Minus(float tti){
        return a - (b * Mathf.Pow(tti, -c));
    }

    protected float getTau1Plus(float tti){
        return a + (b * Mathf.Pow(tti, -c));
    }

    protected float getPhiPlus(){
        List<Rigidbody2D> pPlus = getPPlus();
        float min = Mathf.Infinity;
        foreach (Rigidbody2D point in pPlus) {
            float alphaDerivative = getAlphaDerivative(point);
            float tti = getTTI(point);
            float tau1Plus = getTau1Plus(tti);
            float phi = alphaDerivative - tau1Plus;
            if (phi < min) {
                min = phi;
            }
        }

        if (min == Mathf.Infinity) {
            min = 0.0f;
        }
    
        return min;
    }

    protected float getPhiMinus() {
        List<Rigidbody2D> pMinus = getPMinus();
        float max = Mathf.NegativeInfinity;
        foreach (Rigidbody2D point in pMinus) {
            float alphaDerivative = getAlphaDerivative(point);
            float tti = getTTI(point);
            float tau1Minus = getTau1Minus(tti);
            float phi = alphaDerivative - tau1Minus;
            if (phi > max) {
                max = phi;
            }
        }

        if (max == Mathf.NegativeInfinity) {
            max = 0.0f;
        }

        return max;
    }

    protected float getThetaDerivative() {
        float alphaDerivativeGoal = getAlphaDerivative(goal) * Mathf.Rad2Deg;
        float alphaDerivativeGoalAbs = Mathf.Abs(alphaDerivativeGoal);
        float phiPlus = getPhiPlus() * Mathf.Rad2Deg;
        float phiMinus = getPhiMinus() * Mathf.Rad2Deg;

        float alphaThreshold = 0.1f * Time.deltaTime * Mathf.Rad2Deg;
        if (alphaDerivativeGoalAbs < alphaThreshold) {
            //Paper deviation: if PPlus or PMinus is empty, use the smallest phi thats still gerater zero 
            if (!Mathf.Abs(phiPlus).Equals(0.0f) && !Mathf.Abs(phiMinus).Equals(0.0f)) {
                if (Mathf.Abs(phiPlus) < Mathf.Abs(phiMinus)) {
                    return phiPlus;
                } else {
                    return phiMinus;
                }
            } else {
                if (Mathf.Abs(phiMinus).Equals(0.0f)) {
                    return phiPlus;
                } else {
                    return phiMinus;
                }
            }
        } else if (phiMinus < alphaDerivativeGoal && alphaDerivativeGoal < phiPlus) {
            if(Mathf.Abs(phiPlus - alphaDerivativeGoal) < Mathf.Abs(phiMinus - alphaDerivativeGoal)){
                return phiPlus;
            }else{
                return phiMinus;
            }
        } else if (alphaDerivativeGoal < phiMinus || alphaDerivativeGoal > phiPlus) {
            float alpha = getAlpha(goal); //Paper deviation: here alphaGoal is used instead of alphaDerivativeGoal
            return alpha;
        }else{
            return -1.0f;
        }
    }

    protected List<Rigidbody2D> getPCol() {
        List<Rigidbody2D> pCol = new List<Rigidbody2D>();
        foreach (Rigidbody2D point in p) {
            float alpha = getAlpha(point);
            float tti = getTTI(point);
            float tau1 = getTau1(tti, point);
            //print("alpha: " + alpha + ", tti: " + tti + ", tau1: " + tau1 + ", alphaDerivative: " + getAlphaDerivative(point));
            if (tti > 0.0f && alpha < tau1) {
                pCol.Add(point);
            }
        }
        this.pCol = pCol; //DEBUG
        return pCol;
    }

    protected List<Rigidbody2D> getPPlus() {
        List<Rigidbody2D> pPlus = new List<Rigidbody2D>();
        List<Rigidbody2D> pCol = getPCol();
        foreach (Rigidbody2D point in pCol) {
            float alphaDerivative = getAlphaDerivative(point);
            if (alphaDerivative > 0.0f) {
                pPlus.Add(point);
            }
        }
        return pPlus;
    }

    protected List<Rigidbody2D> getPMinus() {
        List<Rigidbody2D> pMinus = new List<Rigidbody2D>();
        List<Rigidbody2D> pCol = getPCol();
        foreach (Rigidbody2D point in pCol) {
            float alphaDerivative = getAlphaDerivative(point);
            if (alphaDerivative < 0.0f) {
                pMinus.Add(point);
            }
        }
        return pMinus;
    }

    protected List<Rigidbody2D> getPPos() {
        List<Rigidbody2D> pPos = new List<Rigidbody2D>();
        List<Rigidbody2D> pCol = getPCol();
        foreach (Rigidbody2D point in pCol) {
            float tti = getTTI(point);
            if (tti < tau2) {
                pPos.Add(point);
            }
        }
        return pPos;
    }

}

