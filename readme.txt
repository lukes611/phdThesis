Title Pages

2. [now I only mentioned I reference any previous work]

Abstact

4. "3D reconstruction algorithms generate 3D data from image or video data." ==>  should be changed to "Three dimensional reconstruction algorithms generate 3D data from two dimensional image or video data.

5. "Fourier Volume Registration was explored in order to document the effects of 3D reconstruction and registration. ==>  should be changed to "This thesis investigates the application of Fourier Volume Registration to 3D reconstruction".

6. "it is a closed solution" ==>  should be changed to "it is a closed form solution"

7. The abstract should reflect that the plane tree was first proposed in your honours thesis, give a reference your honour's thesis and  then discuss what additional work or improvements you have made over that. 

8.  "The findings presented on both the Fourier Volume Reg- istration method and the Plane-Tree indicate an improvement over existing methods and may lead to new research into the areas of Fourier registration and Hierarchical data representation research." ==> delete the second half of this sentence

Sect 1.2 Contributions

9. "In terms of compression research, a novel data representation based on Octree 3D data compression is proposed" ==> this is confusing since this contribution was made in your honours thesis. State what improvements or modifications have you made since then.

Chapter 2 

10. I stopped looking at the english and just focused on content. I have no specific comments for chap 2

Chapter 3 

11. I find that the names of sections 3.1.1, 3.1.2, and 3.1.3, confuse me more than help me understand the work that each describes. Since sect 3.1 is already entitled "Fourier Volume Reconstruction" I would rename them 
    3.1.1 Basic Volume Reconstruction
    3.1.2 Reconstruction from stereo or 3D data   
    3.1.3 Reconstruction from monocular or 2D data

12. Be sure in include the complexity discussion of SIFT in section 3.14 that I gave you the reference for.

13. Section 3.2 needs to changed in line with comments 2 & 7

Chapter 4 

14. The title of sect 4.4 would be better "Experiments using different data sources"

15.  The title of sect 4.4 would be better "Data Sources". It is not as important how you got the data  but what the source data consists of, i.e. is it monocular, does it have high res depth (laser, calibrated stereo), or low res depth (active camera & disparity map) 

16. The results in 4.4.3 look good but you need to state the specific parameters you used with the FM2D and FM3D and ICP methods. Did you use the default values? What were they? Ie # scale factors used, gaussian window size, descriptor region radius etc.

17. The results in 4.4.4 are a mixed bag, but that is good as well because it shows that with low res depth data the high noise levels means that you are going to get random performance for all methods which is fair enough

18. In sec 4.4.5 there are not a lot of no comparative results it would be nicer to have something more.

19. In table 4.2 it would be good to have a row at the bottom of each set that has the average mean square error across all the different noise values for each method to make it easier to get the picture. Where you have a fail just use and error of 20 or so.

20. ditto for table 4.3

21. You should make some comment in section 4.5 and 4.6 that since you are using the active camera data source that don't give the best results that it affects the performance margin of FVR compared to the FM and ICP methods

22. Figure 4.27 shows reconstructions for FVR but none for ICP and FM you state that they are difficult to reconstruct with these, but a picture is worth 1000 words the worse they look the better it is for you. If you really can not reconstruct these then please given specific and detailed justification of why they are difficult to reconstruct.

23. Please don't start the section with a table, move it towards the end of the section

24. There is no run time comparison to verify your theoretical complexity in sect 3.1.4. Why not?
    Even if you can't give a full comparison against all methods at least give some indication of run-time in milliseconds for your implementation of FVR, FFVR etc. I think it is important to include some actual run times. 

25. In sect 4.8 - 4.12 you should only present results that are novel and compare them against those in your honours thesis. 

Regards



