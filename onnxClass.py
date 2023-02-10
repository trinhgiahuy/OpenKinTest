import argparse
import tarfile
import os
import dill
import torch
import numpy as np
import tqdm
import pandas as pd
import requests
from torchvision.io.image import read_image
from time import time
from tqdm import tqdm
from utils import *

try:
    import onnx
    import onnxruntime as rt
except ImportError as e:
    raise ImportError(f"Please install onnx and onnxruntime first. {e}")

torch.manual_seed(0)

class OnnxClassifier:
    def __init__(self, pre, model, post) -> None:
        self.model = rt.InferenceSession(model)
        self.preProcess = pre
        self.postProcess = post

        self.preProcessTime = 0.0
        self.inferenceTime = 0.0
        self.postProcessTime = 0.0

    def __call__(self, inputImg):
        start = time()
        inputBatch = self.preProcess(inputImg).unsqueeze(0)
        preProcessTime = time()
        self.preProcessTime = preProcessTime - start

        resultRet = self.model.run(None, {"input1": inputBatch.detach().numpy()})
        inferenceTime = time()
        self.inferenceTime = inferenceTime - preProcessTime

        categoryClass, scoreClass = self.postProcess(resultRet)
        postProcessTime = time()
        self.postProcessTime = postProcessTime - inferenceTime

        return categoryClass, scoreClass


if __name__ == "__main__":
    
    print()

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--numIteration", type=int, default=5000, help="Number of iterations to run"
    )
    parser.add_argument(
        "--modelFn", type=str, required=True, help="File name of the model archive"
    )
    args = parser.parse_args()

    numIteration = args.numIteration
    modelFn = args.modelFn
    # print(modelFn)

    imageClassPath = f"{os.getcwd()}/image-class"
    modelFnPath = os.path.join(imageClassPath,modelFn)
    # print(modelFnPath)
    
    file = tarfile.open(modelFnPath)
    print(file.getnames)
    file.extractall(imageClassPath)
    file.close()
    
    # Load ONNX file
    modelName = modelFn.split(".")[0]
    # print(f"modelName:{modelName}")
    # print(modelName)
    # onnxModelPath = './unzip/modelONNX_'+modelName+'.onnx'
    onnxModelName = "modelONNX_" + modelName + ".onnx"
    # print(f"onnxModelName:{onnxModelName}")
    # print(imageClassPath)
    onnxModelPath = os.path.join(imageClassPath,onnxModelName)
    # print(onnxModelPath)
    preProcessName = "preProcess_" + modelName + ".pkl"
    preProcessPath = os.path.join(imageClassPath,preProcessName)
    # print(preProcessPath)
    postProcessName = "postProcess_" + modelName + ".pkl"
    postProcessPath = os.path.join(imageClassPath,postProcessName)

    with open(preProcessPath, "rb") as f1:
        preProcess = dill.load(f1)
    f1.close()

    with open(postProcessPath, "rb") as f2:
        postProcess = dill.load(f2)
    f2.close()

    model = OnnxClassifier(pre=preProcess, model=onnxModelPath, post=postProcess)
    scoreClasstDict = {}
    timeBenchmarkList = []

    # Code for numerical inference
    # for i in range(numIteration):
    #     testInput = torch.rand(pil_img.shape)
    #     classOut, scoreOut = model(testInput)
    #     preProcessTime, inferenceTime, postProcessTime = (
    #         model.preProcessTime,
    #         model.inferenceTime,
    #         model.postProcessTime,
    #     )
    #     scoreClasstDict[classOut] = scoreOut
    #     tempTimeList = [preProcessTime, inferenceTime, postProcessTime]
    #     timeBenchmarkList.append(tempTimeList)
    # -----------------------------------

    # Code for image inference
    imageDataSet = processCIFAR(url=trainURL)
    # print(np.shape(imageDataSet))

    for i in tqdm(range(numIteration)):
        tempImg = imageDataSet[i]
        classOut, scoreOut = model(torch.from_numpy(tempImg.transpose(2, 0, 1)))
        preProcessTime, inferenceTime, postProcessTime = (
            model.preProcessTime,
            model.inferenceTime,
            model.postProcessTime,
        )
        scoreClasstDict[classOut] = scoreOut
        tempTimeList = [preProcessTime, inferenceTime, postProcessTime]
        timeBenchmarkList.append(tempTimeList)

    print("Finishing benchmark!!")

    timeBenchmarkArr = np.array(timeBenchmarkList)
    timeBenchamrkSum = timeBenchmarkArr.sum(axis=0)
    # print(scoreClasstDict)
    # print(timeBenchamrkSum)

    preProTime, inferTime, postProTime = (
        timeBenchamrkSum[0],
        timeBenchamrkSum[1],
        timeBenchamrkSum[2],
    )

    print()
    
    modelOrgName = args.modelFn.split(".")[0]
    print(f"Model {modelOrgName} Inference Result\n")
    print(f"Pre-process time:\t{preProTime} secs\nInference time:\t\t{inferTime} secs\nPost-process time:\t{postProTime} secs")

    csvFileOut = modelName + "_onnx" + ".csv"
    timeBenchmarkDF = pd.DataFrame(timeBenchmarkArr)
    timeBenchmarkDF.to_csv(csvFileOut, header=False, index=False)
    print(f"\nFinish exporting result to file {csvFileOut}")

    os.remove(preProcessPath)
    os.remove(onnxModelPath)
    os.remove(postProcessPath)
