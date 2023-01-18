## Subject 10 8000 epochs

### HR only

![image](https://user-images.githubusercontent.com/65078173/213108991-5da99ff9-0213-4c5f-bd89-8fed22871a32.png)

![image](https://user-images.githubusercontent.com/65078173/213109040-9586c2da-4536-40a9-852e-8aaf5cebde95.png)


```m
>> options

options = 

  TrainingOptionsADAM with properties:

             GradientDecayFactor: 0.9000
      SquaredGradientDecayFactor: 0.9990
                         Epsilon: 1.0000e-08
                InitialLearnRate: 0.0050
               LearnRateSchedule: 'none'
             LearnRateDropFactor: 0.1000
             LearnRateDropPeriod: 10
                L2Regularization: 1.0000e-04
         GradientThresholdMethod: 'l2norm'
               GradientThreshold: Inf
                       MaxEpochs: 8000
                   MiniBatchSize: 128
                         Verbose: 0
                VerboseFrequency: 50
                  ValidationData: []
             ValidationFrequency: 50
              ValidationPatience: Inf
                         Shuffle: 'once'
                  CheckpointPath: ''
             CheckpointFrequency: 1
         CheckpointFrequencyUnit: 'epoch'
            ExecutionEnvironment: 'cpu'
                      WorkerLoad: []
                       OutputFcn: []
                           Plots: 'training-progress'
                  SequenceLength: 'shortest'
            SequencePaddingValue: 0
        SequencePaddingDirection: 'right'
            DispatchInBackground: 0
         ResetInputNormalization: 1
    BatchNormalizationStatistics: 'population'
                   OutputNetwork: 'last-iteration'
                   
 layers = 

  4×1 Layer array with layers:

     1   ''   Sequence Input      Sequence input with 4 dimensions
     2   ''   LSTM                LSTM with 150 hidden units
     3   ''   Fully Connected     1 fully connected layer
     4   ''   Regression Output   mean-squared-error
 >> layers(1)

ans = 

  SequenceInputLayer with properties:

                      Name: ''
                 InputSize: 4
                 MinLength: 1
        SplitComplexInputs: 0

   Hyperparameters
             Normalization: 'rescale-symmetric'
    NormalizationDimension: 'auto'
                       Max: []
                       Min: []

>> layers(2)

ans = 

  LSTMLayer with properties:

                       Name: ''
                 InputNames: {'in'}
                OutputNames: {'out'}
                  NumInputs: 1
                 NumOutputs: 1
             HasStateInputs: 0
            HasStateOutputs: 0

   Hyperparameters
                  InputSize: 'auto'
             NumHiddenUnits: 150
                 OutputMode: 'last'
    StateActivationFunction: 'tanh'
     GateActivationFunction: 'sigmoid'

   Learnable Parameters
               InputWeights: []
           RecurrentWeights: []
                       Bias: []

   State Parameters
                HiddenState: []
                  CellState: []

  Show all properties

>> layers(3)

ans = 

  FullyConnectedLayer with properties:

          Name: ''

   Hyperparameters
     InputSize: 'auto'
    OutputSize: 1

   Learnable Parameters
       Weights: []
          Bias: []

  Show all properties

>> layers(4)

ans = 

  RegressionOutputLayer with properties:

             Name: ''
    ResponseNames: {}

   Hyperparameters
     LossFunction: 'mean-squared-error'
```
