using UnityEngine;
using System.Collections;

public class InstantiateCircleTestcase : MonoBehaviour {

    public Obstacle obstacle;
    public Transform obstacleTarget;
    public Vector2 position;
    public float radius;
    public int amount;

	// Use this for initialization
	void Start () {
        float degreeDistance = 360.0f / (float)amount;
        Vector2 orientation = Vector2.up;
        for (float i = 0.0f; i < 360; i += degreeDistance) {
            float rad = degreeDistance * Mathf.Deg2Rad;
            float sin = Mathf.Sin(rad);
            float cos = Mathf.Cos(rad);

            orientation = new Vector2((cos * orientation.x - sin * orientation.y), (sin * orientation.x + cos * orientation.y)).normalized;
            Quaternion rotation = Quaternion.identity;
            rotation.SetLookRotation(new Vector3(0, 0, 1), new Vector3(-orientation.x, -orientation.y));

            Obstacle o = (Pedestrian)Instantiate(obstacle, position + orientation * radius, rotation);
            Transform ot = (Transform)Instantiate(obstacleTarget, position - orientation * radius, Quaternion.identity);
            o.goal = ot;
        }
	}

    // Update is called once per frame
    void Update() {
	
	}
}
