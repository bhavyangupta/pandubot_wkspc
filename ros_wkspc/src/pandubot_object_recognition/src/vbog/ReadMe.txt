ReadMe to run the Object Recognition
This code will take in an image and return a class label between 1-15 present in the image.

To be Installed:

1. Vlfeat library: 
Download and install from the link: https://pypi.python.org/pypi/pyvlfeat/

2. Scikit library:
Install using the following link: http://scikit-learn.org/stable/install.html

Running intructions:
1. Call the function: "initObject.py"
Purpose: Intialize few parameters
Input: () //No input since its an initialization function
Return: a. ADParams
		b. ResizeAmt
		c. Stride
		d. PatchSize
		e. NumScales
		f. ScaleFactor
		g. SVMModel
		h. TextLabels
		i. km

2. Call the function: "ORLiveVideoWrapper.py"
Purpose: This will return the class labels
Input:  a. frame (1920 x 1080)
		b. ADParams
		c. ResizeAmt
		d. Stride
		e. PatchSize
		f. NumScales
		g. ScaleFactor
		h. SVMModel
		i. TextLabels
		j. km
Note: You just need to take care of the first parameter,i.e. the image. Remaining things will be returned to your function from the previous execution.
Return: a. TextClass (String)
		b. Predicted Probabilities (np.array)


	

