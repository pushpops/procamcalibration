# procamcalibration
camera-projector calibration

The program estimates the internal parameters of the camera and the external parameters of the camera and the projector by inputting several images like the following.

The program estimates the internal parameters of the camera and the external parameters of the camera and the projector by inputting several images like the following

To obtain the following image, a cyan checkerboard pattern is projected from the projector onto a flat plate with a yellow checkerboard pattern printed on it. Several images are then taken by changing the orientation of the flat plate.

If you take more than two pictures, you can perform the estimation, but the accuracy seems to vary with the number of pictures.

Edit the main.cpp file and specify the path to these images.

If you are a Mac user, put the chessboard directory on your desktop, otherwise you need to edit the path in calibration.cpp.

<img src="https://user-images.githubusercontent.com/64456870/133924171-409d244f-5d08-4c12-8a8a-c45e9b288899.JPG" width="480">
