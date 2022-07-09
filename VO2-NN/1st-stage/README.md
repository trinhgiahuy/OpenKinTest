## This is a first stage of summer project:

In 1st stage, we define 7 different NN models for each subjects to find optimal set of parameters (for e.g pattern of speed, acceleration, vertical displacement, change of speed,...) and use more time variant variables that is different from Atiqa's model which used age, height, weight, HR. We want to obtain oxygen uptake during entire measurement instead of maximum uptake. In this stage, we can make our first publication on training NN and testing on the first participant


Lately, we can go to next stage which is the approach 1 above ( we trained 6 participants and predict 7th one which is not part of training dataset)

**Multiple_Subject_Input.ipynb**

- Input train: Subj 1 - 7
- Output predict test: Subj 8
- Fig: multiple_subject_input_pred_7.png
- Model: Multiple_Subject_Input_v1.h5
