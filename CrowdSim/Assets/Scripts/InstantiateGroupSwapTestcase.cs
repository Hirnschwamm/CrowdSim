using UnityEngine;
using System.Collections;

public class InstantiateGroupSwapTestcase : MonoBehaviour {

    public Obstacle obstacle;
    public Transform obstacleTarget;
    public Vector2 position;
    public float distance;
    public int groupColoumns;
    public int groupRows;
    public float obstacleDistance;
   

	// Use this for initialization
	void Start () {
        Vector2 groupPos = position + Vector2.up * (distance/2.0f);
        Quaternion orientation = Quaternion.identity;
        orientation.SetLookRotation(new Vector3(0, 0, 1), new Vector3(0, -1));

        placeGroup(groupPos, orientation);

        groupPos = position - Vector2.up * (distance/2.0f);
        orientation.SetLookRotation(new Vector3(0, 0, 1), new Vector3(0, 1));

        placeGroup(groupPos, orientation);
	}

    private void placeGroup(Vector2 groupPos, Quaternion orientation) {
        Vector2 distanceVector;
        for (int i = 0; i < groupColoumns; i++) {
            for (int j = 0; j < groupRows; j++) {
                distanceVector = groupPos + (-Vector2.left * i * obstacleDistance) + (Vector2.up * j * obstacleDistance);
                Obstacle o = (Pedestrian)Instantiate(obstacle, position + groupPos + distanceVector, orientation);
                Transform ot = (Transform)Instantiate(obstacleTarget, (position - groupPos) * 2.0f + distanceVector, Quaternion.identity);
                o.goal = ot;
            }
        }
    }

    // Update is called once per frame
    void Update() {
	
	}
}
