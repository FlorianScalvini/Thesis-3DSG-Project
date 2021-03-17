using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System.IO;
using System;

[ExecuteInEditMode]
public class DepthScript : MonoBehaviour
{
    const int height = 480; // Hauteure
    const int width = 640; // Largeur

    const int num_disp = 64; // Nombre de disparité
    const int penality_1 = 8;
    const int penality_2 = 128;

    public ComputeShader shaderCT;
    public ComputeShader shaderMC;
    public ComputeShader shaderCAH;
    public ComputeShader shaderCAV;
    public ComputeShader shaderSPA;
    public ComputeShader shaderWTA;
    public ComputeShader shaderPT;
    public ComputeShader shaderBMS;
    public ComputeShader shaderWT;

    int kernelCT;
    int kernelMC;
    int kernelCAH;
    int kernelCAV;
    int kernelSPA;
    int kernelWTA;
    int kernelPT;
    int kernelBMS;

    Camera cameraLeft;
    Camera cameraRight;
    Camera cameraMain;

    ComputeBuffer bufferCT;
    ComputeBuffer bufferMC;
    ComputeBuffer bufferAC;
    ComputeBuffer bufferMinAC;
    ComputeBuffer bufferPT;
    ComputeBuffer bufferWT;

    RenderTexture finalRenTex;

    uint[] arrayCT; // Tableau de coût
    uint[] arrayMC; // Tableau de coût
    uint[] arrayAC; // Tableau de coût
    uint[] arrayMinAC; // Tableau de coût
    uint[] arrayPT; // Tableau de coût
    float[] arrayWT; // Tableau de coût


    void OnApplicationQuit()
    {
        Debug.Log("Application ending after " + Time.time + " seconds");
        //GetComponent<AudioSource>().Stop();
    }




    // Start is called before the first frame update
    void Start()
    {
        // Make the game run as fast as possible
        Application.targetFrameRate = 30;

        cameraMain = GameObject.Find("Main Camera").GetComponent<Camera>();
        cameraLeft = GameObject.Find("CameraLeft").GetComponent<Camera>();
        cameraRight = GameObject.Find("CameraRight").GetComponent<Camera>();

        cameraRight.targetTexture = new RenderTexture(width, height, 24);
        cameraLeft.targetTexture = new RenderTexture(width, height, 24);


        kernelCT = shaderCT.FindKernel("CSCensusTransform");
        kernelMC = shaderMC.FindKernel("CSMatchCost");
        kernelCAH = shaderCAH.FindKernel("CSCostAggregationHorizontal");
        kernelCAV = shaderCAV.FindKernel("CSCostAggregationVertical");
        kernelSPA = shaderSPA.FindKernel("CSSumPathAgree");
        kernelWTA = shaderWTA.FindKernel("CSWinnerTakeAll");
        kernelPT = shaderPT.FindKernel("CSMedianFilter");
        kernelBMS = shaderBMS.FindKernel("CSBlockMatSAD");

        bufferCT = new ComputeBuffer(width * height * 2, 4);
        bufferMC = new ComputeBuffer(width * height * num_disp, 4);
        bufferAC = new ComputeBuffer(width * height * num_disp * 4, 4);
        bufferMinAC = new ComputeBuffer(width * height * 4, 4);
        bufferPT = new ComputeBuffer(width * height, 4);
        bufferWT = new ComputeBuffer(width * height, 4);

        arrayCT = new uint[width * height * 2];
        arrayMC = new uint[width * height * num_disp];
        arrayAC = new uint[width * height * num_disp * 4];
        arrayMinAC = new uint[width * height * 4];
        arrayPT = new uint[width * height];
        arrayWT = new float[width * height];
        initShader();

        finalRenTex = new RenderTexture(width, height, 24);
        finalRenTex.enableRandomWrite = true;
        finalRenTex.Create();

    }



    // Update is called once per frame
    void Update()
    {
        if (Time.frameCount % 30 == 0)
        {
            System.GC.Collect();
        }
        cameraLeft.fieldOfView = cameraMain.fieldOfView;
        cameraRight.fieldOfView = cameraMain.fieldOfView;

        bool takeHiResShot = false;
        takeHiResShot |= Input.GetKeyDown("k");
        if (takeHiResShot)
        {
            RenderTexture rt = new RenderTexture(width, height, 24);
            cameraLeft.targetTexture = rt;
            Texture2D screenShot = new Texture2D(width, height, TextureFormat.RGB24, false);
            cameraLeft.Render();
            RenderTexture.active = rt;
            screenShot.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            cameraLeft.targetTexture = null;
            RenderTexture.active = null; // JC: added to avoid errors
            Destroy(rt);
            byte[] bytes = screenShot.EncodeToPNG();
            print(bytes[0]);
            string filename = string.Format("{0}/screenshots/screen_{1}x{2}_{3}.png",
                             Application.dataPath,
                             width, height,
                             System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss")); ;
            System.IO.File.WriteAllBytes(filename, bytes);
            Debug.Log(string.Format("Took screenshot to: {0}", filename));
            takeHiResShot = false;
        }
    }

    void initShader()
    {
        bufferCT.SetData(arrayCT);
        shaderCT.SetInt("width", width);
        shaderCT.SetInt("height", height);
        shaderCT.SetTexture(kernelCT, "ImgInput", cameraLeft.targetTexture);
        shaderCT.SetTexture(kernelCT, "ImgInput_2", cameraRight.targetTexture);
        shaderCT.SetBuffer(kernelCT, "Result", bufferCT);

        shaderMC.SetInt("width", width);
        shaderMC.SetInt("height", height);
        shaderMC.SetInt("num_disp", num_disp);
        shaderMC.SetBuffer(kernelMC, "Input", bufferCT);
        shaderMC.SetBuffer(kernelMC, "Result", bufferMC);
        shaderMC.SetBuffer(kernelMC, "minPreArray", bufferMinAC);

        shaderCAH.SetInt("width", width);
        shaderCAH.SetInt("height", height);
        shaderCAH.SetInt("num_disp", num_disp);
        shaderCAH.SetInt("penality_1", penality_1);
        shaderCAH.SetInt("penality_2", penality_2);
        shaderCAH.SetBuffer(kernelCAH, "Input", bufferMC);
        shaderCAH.SetBuffer(kernelCAH, "minPreArray", bufferMinAC);
        shaderCAH.SetBuffer(kernelCAH, "Result", bufferAC);
        shaderCAV.SetBuffer(kernelCAV, "Input", bufferMC);
        shaderCAV.SetBuffer(kernelCAV, "minPreArray", bufferMinAC);
        shaderCAV.SetBuffer(kernelCAV, "Result", bufferAC);

        shaderCAV.SetInt("width", width);
        shaderCAV.SetInt("height", height);
        shaderCAV.SetInt("num_disp", num_disp);
        shaderCAV.SetInt("penality_1", penality_1);
        shaderCAV.SetInt("penality_2", penality_2);

        shaderWTA.SetInt("width", width);
        shaderWTA.SetInt("height", height);
        shaderWTA.SetInt("num_disp", num_disp);

        shaderSPA.SetInt("width", width);
        shaderSPA.SetInt("height", height);
        shaderSPA.SetInt("num_disp", num_disp);

        bufferMC.SetData(arrayMC);
        bufferAC.SetData(arrayAC);
        bufferMinAC.SetData(arrayMinAC);

        shaderBMS.SetInt("width", width);
        shaderBMS.SetInt("height", height);
        shaderBMS.SetInt("num_disp", num_disp);


        shaderPT.SetBuffer(kernelPT, "Input", bufferWT);
        shaderPT.SetInt("width", width);
        shaderPT.SetInt("height", height);
        shaderPT.SetInt("num_disp", num_disp);
        shaderPT.SetFloat("fovXbaseline", Math.Abs(cameraRight.transform.localPosition.x - cameraLeft.transform.localPosition.x) * width * 0.5f / (float)(Math.Tan(cameraMain.fieldOfView * 0.5 * Math.PI / 180)));
        shaderSPA.SetBuffer(kernelSPA, "Input", bufferAC);
        shaderSPA.SetBuffer(kernelSPA, "Result", bufferMC);
        shaderWTA.SetBuffer(kernelWTA, "Input", bufferMC);
        shaderWTA.SetBuffer(kernelWTA, "Result", bufferWT);
    }


    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        /*
        shaderBMS.SetTexture(kernelBMS, "ImgLeft", cameraLeft.targetTexture);
        shaderBMS.SetTexture(kernelBMS, "ImgRight", cameraRight.targetTexture);
        shaderBMS.SetBuffer(kernelBMS, "Result", bufferPT);
        shaderBMS.Dispatch(kernelBMS, width / 8, height / 8, 1);
        */

        
        
        computeCensusTransform();
        
        computeMatchingCost();
        
        computeAggregationCost();
        
        computeWinnerTakeAll();
        computePostTraitement();
        
        
        Graphics.Blit(finalRenTex, destination);
    }

    void computeCensusTransform()
    {
        shaderCT.Dispatch(kernelCT, width / 8, height / 8, 2);

    }

    void computeMatchingCost()
    {
        shaderMC.Dispatch(kernelMC, width / 8, height / 8, num_disp);
    }

    void computeAggregationCost()
    {
        for (int index = 0; index < width; index++)
        {
            shaderCAH.SetInt("index", index);
            shaderCAH.Dispatch(kernelCAH, height / 8, num_disp / 8, 2);
        }


        for (int index = 0; index < height; index++)
        {
            shaderCAV.SetInt("index", index);
            shaderCAV.Dispatch(kernelCAV, width / 8, num_disp / 8, 2);
        }
    }

    void computeWinnerTakeAll()
    {
        shaderSPA.Dispatch(kernelSPA, width / 8, height / 8, num_disp);
        shaderWTA.Dispatch(kernelWTA, width / 8, height / 8, 1);
    }

    void computePostTraitement()
    {

        shaderPT.SetTexture(kernelPT, "Result", finalRenTex);
        shaderPT.Dispatch(kernelPT, width / 8, height / 8, 1);
    }

    void OnDestroy()
    {
        System.GC.Collect();
    }
}
