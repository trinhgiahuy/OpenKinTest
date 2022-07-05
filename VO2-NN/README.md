## NORE: SUBJECT 2 HERE MEANS SUBJECT 3


### EvaluateTable

| ModelName               								    | Pred1					    			  |Pred2					    			  
|-------------------------------------------------------------------------------------------|----------------------------------------------------------------------|
|Best_nha_bao_model_bidir.h5			                    |                               |0.3535                                                                |
|ModelTrained-12									        |0.                 			||
|ModelTrained-121                                           |                               |0.2020|



Some important link

[Normalize columns of pandas data frame](https://stackoverflow.com/questions/26414913/normalize-columns-of-pandas-data-frame)

[Solving TensorFlow cuDNN Initialization Failure Problem](https://leimao.github.io/blog/TensorFlow-cuDNN-Failure/)

It has been a few times that I ran into the following error when I was using TensorFlow in the Docker container.

```
Unknown: Failed to get convolution algorithm. This is probably because cuDNN failed to initialize, so try looking to see if a warning log message was printed above.
```
It seems that it is because cuDNN failed to initialize. However, the reasons behind causing this are unknown.

Usually restarting the computer would solve the problem. However, if we don’t want to spend time restarting the computer, there are some quick solutions.

## TensorFlow-Keras 1.x

```
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
tf.keras.backend.set_session(tf.Session(config=config))
```

## TensorFlow 2.x

```
gpu_devices = tf.config.experimental.list_physical_devices('GPU')
for device in gpu_devices:
    tf.config.experimental.set_memory_growth(device, True)
```

## TensorFlow Allow Growth

By default, TensorFlow would use all the GPU memory regardless of the size of the model you are running. 

That is also why we would need to specify the visible GPU devices when we are running the model on a multi-GPU server to prevent collisions with others. To only use the memory required for the model, we set the GPU memory to allow growth.

But it still remains mysterious to me why cuDNN sometimes would fail to initialize if the GPU memory does not allow growth.

## References

[TensorFlow 1.x Bug](https://github.com/tensorflow/tensorflow/issues/24828)

[keras LSTM Fail to find the dnn implementation #36508](https://github.com/tensorflow/tensorflow/issues/36508)

[TensorFlow 2.x - Limiting GPU Memory Growth](https://www.tensorflow.org/guide/gpu#limiting_gpu_memory_growth)

