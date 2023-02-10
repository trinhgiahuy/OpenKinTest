#!/bin/bash

# SETTING FOR EXECUTION
numIteration=1

#  Down load CIFAR dataset
echo "Downloading CIFAR dataset..."
python -c 'from utils import *; print(downloadCIFAR())'

# echo $arrModelList
echo "Executing extract models from Google Drive..."
python -c 'from utils import *; print(downloadImageClass())'


arrModelList=`python -c 'from constant import *; print(" ".join(getImageClassModelList()))'`


for modelName in $arrModelList
do
    modelFn="${modelName}.tar.gz"
    echo "Executing classifier onnx model ${modelFn}"
    python onnxClass.py --modelFn $modelFn --numIteration $numIteration
done


