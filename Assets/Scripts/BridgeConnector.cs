using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BridgeConnector : MonoBehaviour {
    public string Address = "10.84.125.173";
    public const int DefaultPort = 9090;
    public int Port = 9090;
    public Comm.Bridge Bridge { get; private set; }
    SensorLidar1 sl;

    void Awake(){
        Bridge = new Comm.Ros.RosBridge();
        sl = new SensorLidar1();
        if (Bridge.Status == Comm.BridgeStatus.Disconnected)
        {
            Bridge.Connect(Address, Port, 1);
            UnityEngine.Debug.Log("BridgeConnector Awake. Bridge connected to address:port = "+Address+":"+Port);
            sl.OnBridgeAvailable(Bridge);
        }
    }


	// Update is called once per frame
	void Update () {
        Bridge.Update();
	}

    /*
    // Use this for initialization
    void Start()
    {

        UnityEngine.Debug.Log("BridgeConnector Start");
    }

    public void LidarToggle2(bool newValue){
        UnityEngine.Debug.Log("Lidar toggled2: " + newValue);
        //Enable the Lidar component

        //Set the 
        sl.OnBridgeAvailable(Bridge);
    }
    */
}
