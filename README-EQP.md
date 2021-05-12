# 2.183Project
Tennis Serve Modeling Project for MIT Course 2.183 S21

Instructions to get output torque vector (3x300) from equilibrium-point model.

1. Upload "tennis_serve_video.mov" to MATLAB workspace.
2. Run tennisServeModel code to get the actual angle and velocity vectors in the workspace.
3. Tune the stiffness and damping parameters in the eqPoint script.
4. Run eqPoint script. 
5. This will call the getVideoPoints function, which will bring up a figure. It will take you through 8 frames twice.
6. Here, you should click on shoulder --> elbow --> wrist in THAT order, as many times as it prompts you. 
7. Then, the script will calculate the equilibrium angle vector from interpolating from smooth splines fitted to these 8 points from the video. 
8. Last, the script should finish computing and give nOut, the resultant joint torque vector, in the workspace.
9. This nOut vector should then be fed into the inertial mechanics (Forward dynamics) model to get the joint trajectory from the equilibrium point model!
10. 
