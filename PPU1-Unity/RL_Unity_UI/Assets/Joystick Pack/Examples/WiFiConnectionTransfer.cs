using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class WiFiConnectionTransfer: MonoBehaviour
{

    public string Url = "";
    public Text IPstring;
    public VariableJoystick variableJoystick;
    public Text valueText;
    public float X;
    public float Y;

    private IEnumerator SendRequest(string URL)
    {
        var www = new WWW(URL);
        while (!www.isDone)
            yield return www;
    }

    // Use this for initialization
    void Start()
    {

    }

     //Update is called once per frame
    void Update()
    {
        Url = "http://" + IPstring.text + "/JoystickPosition=";
        X = variableJoystick.Horizontal;
        Y = variableJoystick.Vertical;

        if (variableJoystick.Horizontal == 0.0 && variableJoystick.Vertical == 1.0)
        {
            UP();
            Url = Url + "1";
        }

        if (variableJoystick.Horizontal == 0.0 && variableJoystick.Vertical == -1.0)
        {
            DOWN();
            Url = Url + "2";
        }

        if (variableJoystick.Horizontal == -1.0 && variableJoystick.Vertical == 0.0)
        {
            LEFT();
            Url = Url + "3";
        }

        if (variableJoystick.Horizontal == 1.0 && variableJoystick.Vertical == 0.0)
        {
            RIGHT();
            Url = Url + "4";
        }

        if (variableJoystick.Horizontal == 0.0 && variableJoystick.Vertical == 0.0)
        {
            idle();
            Url = Url + "5";
        }

    }

    public void UP()
    {
        StartCoroutine(SendRequest(Url + "1"));
    }

    public void DOWN()
    {
        StartCoroutine(SendRequest(Url + "2"));
    }

    public void LEFT()
    {
        StartCoroutine(SendRequest(Url + "3"));
    }

    public void RIGHT()
    {
        StartCoroutine(SendRequest(Url + "4"));
    }

    public void idle()
    {
        StartCoroutine(SendRequest(Url + "5"));
    }

    public void Exit()
    {
        Application.Quit();
    }

}