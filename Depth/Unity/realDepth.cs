using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class realDepth : MonoBehaviour
{

    Color[] aColors;
    Texture2D texture;
    int width = 640;
    int height = 380;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start");
        StartShader();
    }

    void StartShader()
    {
        Camera camera = GameObject.Find("Main Camera").GetComponent<Camera>();
        camera.depthTextureMode = DepthTextureMode.Depth;
        Shader depthRenderer = Shader.Find("depthShader");
        camera.SetReplacementShader(depthRenderer, "RenderType");
        texture = new Texture2D(width, height);
    }

    // Update is called once per frame
    void Update()
    {
        
    }


    void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Graphics.Blit(source, destination);//for no changes
    }
}
