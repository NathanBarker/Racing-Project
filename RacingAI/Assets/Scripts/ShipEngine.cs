using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.WSA;

public class ShipEngine : MonoBehaviour
{
    #region Debug

    /* Debug */
    [FormerlySerializedAs("DEBUG_newSteer")] [Header("Debug Variables")] 
    
    public float debugNewSteer = 0.0f;
    public float debugCurrentNode = 0.0f;

    [Space(10)]

    #endregion
    
    [SerializeField] private Rigidbody rigidBody;

    #region Driving Parameters

    /* Driving Parameters */
    [Header("Driving Parameters Variables")] [SerializeField]
    private float distanceToggle;

    [SerializeField] private float turningSpeed;
    [SerializeField] private float maxSteerAngle;
    [SerializeField] private float force;
    [SerializeField] private float maxTiltAngle;

    [Space(10)]

    #endregion

    #region Acceleration and Braking

    /* Acceleration and Braking */
    [Header("Acceleration and Braking Variables")]
    [SerializeField]
    private float acceleration;

    [SerializeField] private float braking;
    [SerializeField] private float brakeThreshold;
    [SerializeField] private bool isBraking = false;

    [Space(10)]

    #endregion

    #region Hover

    /* Hover */
    [SerializeField]
    public Transform[] springs;

    [SerializeField] private float desiredHoverHeight;

    float hoverDamp = 0.2f; // Amount that the lifting force is reduced per unit of upward speed.
    float hoverForce = 30.0f; // Force applied per unit of distance below the desired height.

    #endregion

    #region Path
    
    public Transform pathToFollow;
    
    private int _currentNode = 0;
    private Vector3 _currentRotation;
    private List<Transform> _nodes;

    #endregion

    void Start()
    {
        Transform[] pathTransforms = pathToFollow.GetComponentsInChildren<Transform>();
        _nodes = new List<Transform>();
        foreach (Transform child in pathTransforms)
        {
            if (child != pathToFollow.transform)
            {
                _nodes.Add(child);
            }
        }
    }

    void FixedUpdate()
    {
        ApplySteer();
        Drive();
        CheckDistanceToNextNode();
        HoverShip();
        Balance();
        Tilt();
    }

    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(_nodes[_currentNode].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;

#if UNITY_EDITOR
        debugNewSteer = newSteer;
#endif

        isBraking = newSteer > brakeThreshold || newSteer < -brakeThreshold;

        rigidBody.AddTorque(transform.TransformDirection(Vector3.up) * newSteer * turningSpeed);
        rigidBody.AddTorque(transform.TransformDirection(Vector3.right) * (newSteer * 0.4f));
    }

    private void Drive()
    {
        if (isBraking)
        {
            rigidBody.drag = braking;
        }
        else
        {
            rigidBody.AddForce(transform.TransformDirection(Vector3.forward) * acceleration, ForceMode.Acceleration);
        }
    }

    private void CheckDistanceToNextNode()
    {
        if (Vector3.Distance(transform.position, _nodes[_currentNode].position) < distanceToggle)
        {
            _currentNode = _currentNode == _nodes.Count - 1 ? 0 : _currentNode + 1;
#if UNITY_EDITOR
            debugCurrentNode = _currentNode;
#endif
        }
    }

    private void HoverShip()
    {
        RaycastHit hit;
        Ray downRay = new Ray(transform.position, -Vector3.up);
        // Cast a ray straight downwards.
        if (Physics.Raycast(downRay, out hit))
        {
            // The "error" in height is the difference between the desired height
            // and the height measured by the raycast distance.
            float hoverError = desiredHoverHeight - hit.distance;

            // Only apply a lifting force if the object is too low (ie, let
            // gravity pull it downward if it is too high).
            if (hoverError > 0)
            {
                // Subtract the damping from the lifting force and apply it to
                // the rigidbody.
                float upwardSpeed = rigidBody.velocity.y;
                float lift = hoverError * hoverForce - upwardSpeed * hoverDamp;
                rigidBody.AddForce(lift * Vector3.up);
            }
        }
    }

    private void Balance()
    {
        _currentRotation = transform.eulerAngles;
        _currentRotation.z = Mathf.Lerp(_currentRotation.z, 0, force);
        _currentRotation.x = Mathf.Lerp(_currentRotation.x, 0, force);
        transform.eulerAngles = _currentRotation;
    }

    private void Tilt()
    {
        
    }
}