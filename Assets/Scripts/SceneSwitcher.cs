using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneSwitcher : MonoBehaviour {

    public void GotoLidarSimScene()
    {
        SceneManager.LoadScene("LidarSimScene");
    }

    public void GotoMenuScene()
    {
        SceneManager.LoadScene("MenuScene");
    }
}
