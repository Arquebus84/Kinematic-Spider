using System;
using UnityEditor;
using UnityEngine;
using static UnityEditor.Progress;

public class IKMultiLeg : MonoBehaviour
{
    //Parameters
    [Header("Parameters")]

    //Base Foot; The rest of the chain will be the foot base's parent nodes
    public Transform FootBase;

    //Chainlength of bones
    public int chainLength;

    //Target and pole
    public Transform target;
    public Transform pole;

    //Solver iterations per update
    [Header("Solver Parameters")]
    public int solverIterations = 10;

    //Distance when solver stops
    public float delta = 0.001f;

    //Strength of returning to starting position
    [Range(0, 1)]
    public float snapBackStrength = 1f;

    //System-related parameters
    protected float[] BonesLength;  //Target to origin
    protected float CompleteLength;
    protected Transform[] Bones;
    protected Vector3[] Positions;

    //Model-related parameters (geometry rotation with respect to the system's bone's rotation)
    protected Vector3[] startDirectionSucc;
    protected Quaternion[] startRotationBone;
    protected Quaternion startRotationTarget;
    protected Quaternion startRotationRoot;

    // Update is called once per frame, FixedUpdate is called before Update, but will be called a set number of times
    void Awake()       //Used Awake(), but needed more legs, use FixedUpdate to make it more controlled per frame
    {
        Init();
    }

    void Init()
    {
        //      Initialize the arrays
        // System Arrays
        Bones = new Transform[chainLength + 1];
        Positions = new Vector3[chainLength + 1];
        BonesLength = new float[chainLength];

        //Model-related arrays
        startDirectionSucc = new Vector3[chainLength + 1];
        startRotationBone = new Quaternion[chainLength + 1];

        //      Initialize fields
        if (target == null)
        {
            target = new GameObject(gameObject.name + " Target").transform;
            target.position = FootBase.transform.position;                          //FootBase is the transform not this* transform
        }
        startRotationTarget = target.rotation;
        CompleteLength = 0;

        //Initialize data
        var current = FootBase.transform;                                           //FootBase is the transform not this* transform
        for (var i = Bones.Length - 1; i >= 0; i--)
        {
            Bones[i] = current;
            startRotationBone[i] = current.rotation;

            if (i == Bones.Length - 1)
            {
                //Leaf
                startDirectionSucc[i] = (target.position - current.position); 
            }
            else
            {
                //Middle Bones
                startDirectionSucc[i] = Bones[i + 1].position - current.position;

                BonesLength[i] = startDirectionSucc[i].magnitude;
                CompleteLength += BonesLength[i];
            }

            current = current.parent;
        }

        if (Bones == null)
        {
            throw new UnityException("Chain length value is longer than ancestor chain");
        }
    }

    void LateUpdate()
    {
        ResolveIK();
    }

    private void ResolveIK()
    {
        if (target == null)     //Make sure to not get NullPointerExceptions
            return;

        if (BonesLength.Length != chainLength)
            Init();

        //Get the positions
        for (int i = 0; i < Bones.Length; i++)
            Positions[i] = Bones[i].position;

        var rootRot = (Bones[0].parent != null) ? Bones[0].parent.rotation : Quaternion.identity;
        var rootRotDifference = rootRot * Quaternion.Inverse(startRotationRoot);

        //Check if distance is possible to reach
        if ((target.position - Bones[0].position).sqrMagnitude >= CompleteLength * CompleteLength)
        {
            //Stretch the bones if it's greater
            var direction = (target.position - Positions[0]).normalized;

            //Set everything as a subset of the root bone (start at position 0 + 1)
            for (int i = 1; i < Positions.Length; i++)
                Positions[i] = Positions[i - 1] + direction * BonesLength[i - 1];
        }
        else
        {
            for (int i = 0; i < Positions.Length - 1; i++)
                Positions[i + 1] = Vector3.Lerp(Positions[i + 1], Positions[i] + rootRotDifference * startDirectionSucc[i], snapBackStrength);

            for (int iteration = 0; iteration < solverIterations; iteration++)
            {
                //For both forwards and backwards, normalize the distance

                //Going back (backwards kinematics)
                for (int i = Positions.Length - 1; i > 0; i--)
                {
                    if (i == Positions.Length - 1)
                        Positions[i] = target.position; //Set it to the target
                    else
                        Positions[i] = Positions[i + 1] + (Positions[i] - Positions[i + 1]).normalized * BonesLength[i]; //Set in line on distance
                }

                //Forwards kinematics
                for (int i = 1; i < Positions.Length; i++)
                    Positions[i] = Positions[i - 1] + (Positions[i] - Positions[i - 1]).normalized * BonesLength[i - 1];

                //If position is close enough to the 10 solverIterations (in the editor), then stop the loop
                if ((Positions[Positions.Length - 1] - target.position).sqrMagnitude < delta * delta)
                    break;
            }
        }

        //Move towards the pole
        if (pole != null)
        {
            for (int i = 1; i < Positions.Length - 1; i++)
            {
                var plane = new Plane(Positions[i + 1] - Positions[i - 1], Positions[i - 1]);
                var projectedPole = plane.ClosestPointOnPlane(pole.position);
                var projectedBone = plane.ClosestPointOnPlane(Positions[i]);
                var angle = Vector3.SignedAngle(projectedBone - Positions[i - 1], projectedPole - Positions[i - 1], plane.normal);
                Positions[i] = Quaternion.AngleAxis(angle, plane.normal) * (Positions[i] - Positions[i - 1]) + Positions[i - 1];
            }
        }

        //Set the positions (vice versa) and the rotations
        for (int i = 0; i < Positions.Length; i++)
        {
            if (i == Positions.Length - 1)
                Bones[i].rotation = target.rotation * Quaternion.Inverse(startRotationTarget) * startRotationBone[i];
            else
                Bones[i].rotation = Quaternion.FromToRotation(startDirectionSucc[i], Positions[i + 1] - Positions[i]) * startRotationBone[i];

            Bones[i].position = Positions[i];
        }
    }

    private void OnDrawGizmos()
    {
        if (FootBase == null)           //Make sure that a null Pointer exception (UnassignedReferenceException) is not caught
        {
            return;
        }
        else
        {
            var current = FootBase.transform;
            for (int i = 0; i < chainLength && current != null && current.parent != null; i++)
            {
                var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
                Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(
                    Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
                Handles.color = Color.green;
                Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
                current = current.parent;
            }
        }
    }
}
