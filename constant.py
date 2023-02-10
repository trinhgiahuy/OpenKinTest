# imageClassModelName = [
#     "alexnet",
#     "resnet18", 
#     "resnet50",
#     "mobilenet_v2",
#     "mobilenet_v3_small",
#     "mobilenet_v3_large",
#     "vgg11",
#     "vgg16"]

imageClassModelName = [
    # "alexnet",
    # "resnet18",
    "resnet50",
    "resnet101",
    "resnet152",
    "resnext50_32x4d",
    "resnext101_32x8d",
    "resnext101_64x4d",
    "googlenet",
]

modelLinkDrive = {
    "alexnet": 'https://drive.google.com/u/2/uc?id=11Bhbq7lYO_JQddZRyvYlL7MppEITOqwZ',
    "resnet18": 'https://drive.google.com/u/2/uc?id=1uZ1hrYUY_qLWRsgbvZ0dJgWQq3MtitT_&export=download', 
    "resnet50": 'https://drive.google.com/u/2/uc?id=18NgoVBXIRDQBkpGrX3jEegUBtmo1NOFb' ,
    "resnet101": 'https://drive.google.com/u/2/uc?id=1DtsZJQ6nRySXvWrqIvpLqI_SYb7viQyw',
    "resnet152": 'https://drive.google.com/u/2/uc?id=1d9EMIjq8O-DcFmv8h1vmtXlQlNjA_uHK',
    "resnext50_32x4d": 'https://drive.google.com/u/2/uc?id=1iAeBD7-C3eFLZwTf3UgJphkI4cp6OFCa',
    "resnext101_32x8d": 'https://drive.google.com/u/2/uc?id=1nfcV_hS7pinXWQZtzBjJjXVGqEH-__DW',
    "resnext101_64x4d":'https://drive.google.com/u/2/uc?id=1F9zC8dtMUjLOpmjrZKrc3sNzOg4shzoP',
    "googlenet": 'https://drive.google.com/u/2/uc?id=1SRDSzeYaum4IdieNKWOm_zaRr7Hsadql',
    "mobilenet_v2": '' ,
    "mobilenet_v3_small": '' ,
    "mobilenet_v3_large": '',
    "vgg11": '',
    "vgg16": '',
}

def getImageClassModelList():
    return imageClassModelName