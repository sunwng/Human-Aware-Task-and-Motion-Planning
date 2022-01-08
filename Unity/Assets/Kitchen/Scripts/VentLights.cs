using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VentLights : MonoBehaviour {
    public KeyCode toogleLightKey;
    public Light[] lights;
    void Update() {
        if(Input.GetKeyDown(toogleLightKey))
        foreach (Light light in lights) {
            light.enabled = !light.enabled;
        }
    }
}
