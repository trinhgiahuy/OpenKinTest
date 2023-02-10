import os
import tarfile
import gdown
import requests
from constant import *

cwd = os.getcwd()
cifarDir = cwd + "/cifar-100-python/"
cifarExist = os.path.exists(cifarDir)
trainURL = cwd + "/cifar-100-python/train"

def unpickle(file):
    import pickle

    with open(file, "rb") as fo:
        dict = pickle.load(fo, encoding="latin1")

    return dict


def processCIFAR(url):
    data = unpickle(url)
    imageSet = data["data"]
    imageSet = imageSet.reshape(len(imageSet), 3, 32, 32)
    imageSet = imageSet.transpose(0, 2, 3, 1)

    return imageSet


def downloadCIFAR():
    
    if not cifarExist:
        
        print("CIFAR100 directory is not exist. Creating...")
        print(cifarDir)
        os.mkdir(cifarDir)

        print("CIFAR-100 dataset is not available! Downloading...")
        # Downloading CIFAR data set if it is not available
        CIFAR100Link = "https://www.cs.toronto.edu/~kriz/cifar-100-python.tar.gz"
        CIFAR100TarF = "cifar-100-python.tar.gz"

        response = requests.get(CIFAR100Link, stream=True)
        # if response.status_code == 200:
        #     with open(CIFAR100TarF, "wb") as f:
        #         print("Downloading CIFAR100 tar file ...")
        #         total_size = int(response.headers.get("Content-Length", 0))
        #         progress = tqdm(total=total_size, unit="B", unit_scale=True)
        #         for data in response.iter_content(1024):
        #             print(data)
        #             progress.update(len(data))
        #             f.write(data)
        #         progress.close()
        #     f.close()

        if response.status_code == 200:
            with open(CIFAR100TarF, "wb") as f:
                print("Downloading CIFAR100 tar file ...")
                f.write(response.raw.read())

        CIFAR100File = tarfile.open(CIFAR100TarF)
        print(f"CIFAR Folder Name: cifar-100-python. Extracting ...")
        CIFAR100File.extractall(cwd)
        CIFAR100File.close()
        print(f"Finish extracting")
    else:
        print(f"Found CIFAR-100 dataset in {trainURL}. Start benchmarking!!")

def downloadImageClass():

    for name in imageClassModelName:

        print(f"Extracting model: {name} ...")
        imageClassificationModelsURL = modelLinkDrive[name] 
        print(imageClassificationModelsURL)

        imageClassPath = f"{os.getcwd()}/image-class"
        imageClassTar = name + '.tar.gz'

        if not os.path.exists(imageClassPath):
            print(f"Directory {imageClassPath} not found. Creating ...")
            os.mkdir(imageClassPath)
        else:
            print("Directory already exists!!")

        imageClassFile = os.path.join(imageClassPath,imageClassTar)
        if not os.path.isfile(imageClassFile):
            gdown.download(imageClassificationModelsURL,imageClassFile, quiet=False)
        else:
            print(f"File {imageClassTar} already exists!!")

        print(f"-----------------------------------")

objectDetectionModelsURL = ''
objectDetTar = 'object-detection'