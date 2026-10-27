using System.Collections;
using UnityEngine;

public class RGBCameraPublisher : BaseCameraPublisher
{
    public int FPS = 15;
    public bool EnableOffscreenRendering = true;

    public int width = 640;
    public int height = 480;

    private RenderTexture renderTexture;
    private Texture2D texturePixels;
    private Camera offscreenCamera;

    new void Start()
    {
        base.Start();
        //print("SystemInfo.supportsAsyncGPUReadback: " + SystemInfo.supportsAsyncGPUReadback);
        renderTexture = new RenderTexture(width, height, 24);
        texturePixels = new Texture2D(width, height, TextureFormat.RGBA32, false);
        // Can't have 2 cameras on a single gameobject, so make a child
        GameObject offscreenCameraGO = new GameObject("OffscreenCamera");
        offscreenCameraGO.transform.parent = transform;
        offscreenCameraGO.transform.localPosition = Vector3.zero;
        offscreenCameraGO.transform.localRotation = Quaternion.identity;
        // Add the offscreen camera (a copy of the onscreenCamera)
        offscreenCamera = SEAN.Util.Unity.CopyComponent(camera, offscreenCameraGO);
        offscreenCamera.rect = new Rect(0f, 0f, 1f, 1f);
        offscreenCamera.targetTexture = renderTexture;
        offscreenCamera.aspect = (float)width / height;
        StartCoroutine("OffscreenRendering");
    }

    protected override Camera CalibrationCamera => offscreenCamera;

    IEnumerator OffscreenRendering()
    {
        while (true)
        {
            yield return new WaitForEndOfFrame();

            // render
            RenderTexture prevActiveRT = RenderTexture.active;
            RenderTexture.active = renderTexture;
            offscreenCamera.Render();
            RenderTexture.active = prevActiveRT;

            byte[] byteArray;
            if (SystemInfo.supportsAsyncGPUReadback && EnableOffscreenRendering)
            {
                UnityEngine.Rendering.AsyncGPUReadbackRequest request = UnityEngine.Rendering.AsyncGPUReadback.Request(renderTexture, 0, TextureFormat.RGBA32);
                while (!request.done)
                {
                    yield return new WaitForEndOfFrame();
                }
                if (request.hasError)
                {
                    RenderTexture.active = prevActiveRT;
                    yield return new WaitForSeconds(1.0f / FPS);
                    continue;
                }
                texturePixels.LoadRawTextureData(request.GetData<byte>());
                texturePixels.Apply();
                byteArray = texturePixels.EncodeToPNG();
            }
            else
            {
                // read offscreen texture contents into the CPU readable texture
                RenderTexture.active = renderTexture;
                texturePixels.ReadPixels(new Rect(0, 0, texturePixels.width, texturePixels.height), 0, 0);
                texturePixels.Apply();
                byteArray = texturePixels.EncodeToPNG();
            }

            // restore
            RenderTexture.active = prevActiveRT;

            RosMessageTypes.Sensor.CompressedImageMsg imageMessage = new RosMessageTypes.Sensor.CompressedImageMsg(new RosMessageTypes.Std.HeaderMsg(), "png", byteArray);
            PublishWithCameraInfo(imageMessage);

            yield return new WaitForSeconds(1.0f / FPS);
        }
    }
}
