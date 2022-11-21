**Note**: For subject `15`, we do not slice HR index according to oxygen resampling length

## In this version the HR is linearized using linear interpolate built-in Matlab function call `interp1`. 

But it was resampling with 400Hz rate. 

The plot when it plot together with **V_lon** would be

![image](https://user-images.githubusercontent.com/65078173/201931713-69a7a5b1-3fc4-4f0d-8955-b083dc00722a.png)

The `run.sh` is updated with the following changes:
* `cell_array_parser_vn200` slices V_lon based on  heart rate length
* `velSeg` use steps segmented for taking mean value of HR 
```
stepHR = HR_linear(steps(i):steps(i+1));
HRTest = [HRTest, mean(stepHR)];
```
* `input_features` takes sequence from steps
