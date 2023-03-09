using System.Collections; // borshh
using System.Collections.Generic;
using UnityEngine;
public class Path : MonoBehaviour
{
    public Color lineColor;
    private List<Transform> nodes = new List<Transform>();
    private void OnDrawGizmos()
    {
        Gizmos.color = lineColor;

        Transform[] pathTransforms = GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for(int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
        for (int i = 0; i < nodes.Count; i++)
        {
            Vector3 currentNode = nodes[i].position; //position of current node
            Vector3 previousNode = Vector3.zero;

            if (i > 0)
            {
                previousNode = nodes[i - 1].position; //position of previous node
            } else if (i == 0 && nodes.Count > 1) //the previous node is the last in the list
            {
                previousNode = nodes[nodes.Count - 1].position;
            }

            Gizmos.DrawLine(previousNode, currentNode); //draws the path that the AI will use as a guideline
            Gizmos.DrawWireSphere(currentNode, 1f); //shows the current nodes
        }
    
    }
}