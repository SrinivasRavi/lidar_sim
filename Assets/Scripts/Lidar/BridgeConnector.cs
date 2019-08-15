using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BridgeConnector : MonoBehaviour {
    public string Address = "10.84.125.173";
    public const int DefaultPort = 9090;
    public int Port = 9090;
    public Comm.Bridge Bridge { get; private set; }
    LidarSensor ls;

    void Awake(){
        Bridge = new Comm.Ros.RosBridge();
        ls = new LidarSensor();
        if (Bridge.Status == Comm.BridgeStatus.Disconnected)
        {
            Bridge.Connect(Address, Port, 1);
            ls.OnBridgeAvailable(Bridge);
        }
    }

	// Update is called once per frame
	void Update () {
        Bridge.Update();
	}

    //TODO: Study if and when RosBridge.Disconnect needs to be invoked.
}
