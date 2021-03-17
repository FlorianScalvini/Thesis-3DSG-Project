using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScriptMono : MonoBehaviour
{
    public ComputeShader sobelShader;

    int width = 640;
    int height = 480;
    int kernelHandle;


    // Start is called before the first frame update
    void Start()
    {
        kernelHandle = sobelShader.FindKernel("CSSobel");
        sobelShader.SetInt("width", width);
        sobelShader.SetInt("height", height);
    }

    // Update is called once per frame
    void Update()
    {
        if (Time.frameCount % 30 == 0)
        {
            System.GC.Collect();
        }
    }


    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
    
        sobelShader.SetTexture(kernelHandle, "ImgInput", source);

        RenderTexture sobelRender = new RenderTexture(width, height, 24);
        sobelRender.enableRandomWrite = true;
        sobelRender.Create();

        sobelShader.SetTexture(kernelHandle, "Result", sobelRender);
        sobelShader.Dispatch(kernelHandle, width / 8, height / 8, 1);
     
        Graphics.Blit(source, destination);
        DestroyImmediate(sobelRender);

    }



}
