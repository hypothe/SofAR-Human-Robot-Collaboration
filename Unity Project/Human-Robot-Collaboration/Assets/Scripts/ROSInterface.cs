using JointState = RosMessageTypes.Sensor.JointState;
using RosMessageTypes.HumanBaxterCollaboration;
using UnityEngine;

public class ROSInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;
    
    // Variables required for ROS communication
    public string trajectoryTopicName = "baxter_moveit_trajectory";
    public string stopTrajectoryTopicName = "baxter_moveit_stop_trajectory";
    public string unityTfTopicName = "unity_tf";
    public string jointStateTopicName = "baxter_joint_states";
    public string gripperLTopicName = "robot/limb/left/left_gripper";
    public string gripperRTopicName = "robot/limb/right/right_gripper";

    public GameObject baxter;
    public GameObject avatar;

    private BaxterController controller;
    private TFManager tfManager;

    private bool simStarted;
    private float timeElapsedTf;
    private float timeElapsedJS;
    private float publishTfFrequency;
    private float publishJSFrequency;

    void Start()
    {
        simStarted = false;
        timeElapsedTf = 0;
        timeElapsedJS = 0;
        publishTfFrequency = 0.05f;
        publishJSFrequency = 1.0f;

        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<BaxterController>();
        controller.Init(baxter);

        // Subscribe to MoveIt trajectory topic
        ros.Subscribe<BaxterTrajectory>(trajectoryTopicName, controller.TrajectoryResponse);
		// Subscribe to stop trajectory event
        ros.Subscribe<BaxterStopTrajectory>(stopTrajectoryTopicName, controller.StopTrajectoryResponse);
		// Subscribe to gripper
        ros.Subscribe<BaxterGripperOpen>(gripperLTopicName, controller.GripperLResponse);
        ros.Subscribe<BaxterGripperOpen>(gripperRTopicName, controller.GripperRResponse);

        // Instantiate TF Manager component
        tfManager = gameObject.AddComponent<TFManager>();
        tfManager.Init(avatar, baxter.transform.Find("ground"));

    }

    public void RestCommand()
    {
        controller.GoToRestPosition("both");
        simStarted = true;
    }

    public void Update()
    {
        if (simStarted)
        {
            timeElapsedTf += Time.deltaTime;
            timeElapsedJS += Time.deltaTime;

            if (timeElapsedTf > publishTfFrequency)
            {
                UnityTf unityTfMsg = tfManager.GetUnityTfMessage();
                ros.Send(unityTfTopicName, unityTfMsg);

                timeElapsedTf = 0;
            }

            if (timeElapsedJS > publishJSFrequency)
            {
                JointState jointStateMsg = controller.GetBaxterJointState();
                ros.Send(jointStateTopicName, jointStateMsg);

                timeElapsedJS = 0;
            }
            
        }
    }
}