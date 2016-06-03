Camera Node for E-Series depth camera from Fotonic
=========================

#About
This package creates an image publisher for the E-Series 3D camera. It comes with the associated library for dealing with connecting. I will add notes about E-Series connectivity at a later date when I've actually tested it with the camera. (Oh yeah, I don't know if it works yet). 

#Some installation notes

After building the project, the executable may not be able to find the linked library that I've packaged with the catkin package. In my case, doing ```$ sudo ldconfig ```. As far as I can tell, this should be run by the build process, but it isn't. 
