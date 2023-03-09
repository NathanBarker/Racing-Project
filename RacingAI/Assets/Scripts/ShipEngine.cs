using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipEngine : MonoBehaviour
{
    [SerializeField] public float distanceToggle; //checks whether the AI should start braking or not
    public float turningSpeed; // Speed in which the AI turns on the Y-axis
    public float maxSteerAngle; //The maximum angle the AI can rotate/steer
    public float force; // Force for balancing the ship

    public Transform path; // The transform of the path the AI follows 
    private List<Transform> nodes; //all positions on the path
    private int currentNode = 0; //current position of AI on path
    
    public float speed; // Speed force acting on the AI
    public float brake; // Braking power acting on the AI

    public bool isBraking; // Bool to dictate whether the AI is braking or not

    float eulerAngZ;
    Vector3 currentRotation;


    // Rigidbody component of the AI
    public Rigidbody rb;

    //Hover
    public Transform[] springs;

    public float hoverHeight; // Desired hovering height.
    float hoverForce = 20.0f; //The force applied per unit of distance below the desired height.
    
    float hoverDamp = 0.2f; // The amount that the lifting force is reduced per unit of upward speed.
                            // This damping tends to stop the object from bouncing after passing over
                            // something.

                            //Start is called before the first frame update
    //As soon as the scene starts, follow path
    void Start()
    {
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>(); //ensures it is a new empty list at the beginning

        for (int i = 0; i < pathTransforms.Length; i++) //loops through each transform in pathTransforms
        {
            if (pathTransforms[i] != path.transform) //check to see the relation of the transform
            {
                nodes.Add(pathTransforms[i]); //add to list if not
            }
        }

        isBraking = false;
       
    }

    void FixedUpdate()
    {
        ApplySteer(); //rotates the AI to face the next node on the path
        Balance(); // Balance the AI on the Z-Axis
        Drive(); //moves the AI forward
        CheckDistance(); //checks distance between the AI and nodes to    
        HoverShip(); //(Aesthetic purpose... for now) - allows the AI ship to hover
    }

    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currentNode].position); //points to current node
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle; //length of Vector

        rb.AddTorque(transform.TransformDirection(Vector3.up) * newSteer * turningSpeed); // Applies force to turn the AI on the Y-axis in accordance to the current node
        rb.AddTorque(transform.TransformDirection(Vector3.right) * (newSteer * 0.4f)); // Applies force to turn the AI on the Z-axis in accordance to the current node
    }

    private void Drive()
    {
        if (isBraking)
        {
            rb.drag = brake;
        }
        
        if(!isBraking)
        {
            rb.AddForce(transform.TransformDirection(Vector3.forward) * speed);
        }
    }

    private void CheckDistance() //checks to see how far away te AI is from the next node
    { 
        if (Vector3.Distance(transform.position, nodes[currentNode].position) < distanceToggle)
        {
            // If the curreent node is the last node in the list restart 
            if(currentNode == nodes.Count -1) {
                currentNode = 0;
        } else
            currentNode++; //move to next node/position
        }
    }

    // When the AI collides with a node, the braking bool is true
    private void OnTriggerEnter(Collider Node)
    {
         isBraking = true;
    }

    // When the AI leaves the collider of the node, the braking bool is false
    private void OnTriggerExit(Collider Node)
    {
         isBraking = false;
    }

    private void HoverShip()
    {
        foreach (Transform spring in springs)
        {
            RaycastHit hit;
            Ray downRay = new Ray(transform.position, -Vector3.up);

            // Cast a ray straight downwards.
            if (Physics.Raycast(downRay, out hit))
            {
                // The "error" in height is the difference between the desired height
                // and the height measured by the raycast distance.
                float hoverError = hoverHeight - hit.distance;

                // Only apply a lifting force if the object is too low (ie, let
                // gravity pull it downward if it is too high).
                if (hoverError > 0)
                {
                    // Subtract the damping from the lifting force and apply it to
                    // the rigidbody.
                    float upwardSpeed = rb.velocity.y;
                    float lift = hoverError * hoverForce - upwardSpeed * hoverDamp;
                    rb.AddForce(lift * Vector3.up);
                }
            }
        }
    }

    public void Balance()
    {
        currentRotation = transform.eulerAngles;


        // Balancing for when the Ai ship is leaning over 
        currentRotation.z = Mathf.Lerp(currentRotation.z, 0, force);
        currentRotation.x= Mathf.Lerp(currentRotation.x, 0, force);

        transform.eulerAngles = currentRotation;
    }

}

