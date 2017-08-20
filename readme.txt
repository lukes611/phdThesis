Title Pages
	notes:
		2 - now I only mentioned I reference any previous work

Sect 1.2 Contributions
	notes:
		9 - it is a new algorithm compared to the honours though

Chapter 3 
	TODO
		12. Be sure in include the complexity discussion of SIFT in section 3.14 that I gave you the reference for.
			sift complexity 
				N = N^2 is the no. pixels in the image
				w = gausian filter size  eg 5
				s = number of octaves eg. 3
				α,a = the fraction of extrema in the image [0-1] eg .006
				β,b = fraction of extrema which are features [0-1] eg .35
				γ,y = additional keypoints found after rotation is investigated, eg .0004
				x = the size of the descriptor neighbourhood eg 8

				number key-points = (αβ + γ)(N^2)
				number of operations = complexity = N^2(s(4w^2 + 100a + 156) + 1520x^2(ab + y))
				number of operations per feature (f=num features) = N^2(1520fx^2 + 48s) + 100sf 
				
				complexity given feature percentage: f
				feature percentage is 
				N^2(s(4w^2 + 100*0.006 + 156) + 1520x^2f)
				
			surf complexity
				from: https://github.com/herbertbay/SURF
				N = N^2 is number of pixels
				s = num octaves, eg 3
				u = sample scalar eg 3
				f = fraction of pixels which are features
					create integral image: N^2 + N ops
					fast hessian: [668(N/u*2^i)^2 for i in range(1,s)]
					compute descriptors: 213434 * fN^2 
			pca complexity
				from : http://statweb.stanford.edu/~imj/WEBLIST/AsYetUnpub/sparse.pdf
				N = N^2 is number of pixels
				p = 3, the number of coordinates in 3 space
				= O(p^2N^2+p^3)
				= O(9N^2 + 27)
			ransac complexity
				I = num iterations eg 500
				i = num feature matches eg 200
				
				I * (i * (26 + 108))
				
			3d fm complexity
				similar to 2d
			icp complexity
				I * knn * ([leastsq]12i + 4i + 96) * i*21[transforms]
			k-nearest neighbours:
				n is number of features matched
				k is the number of features matched
				d is 3, for 3 space
				O(ndk)

		13. Section 3.2 needs to mention my honours thesis
			

Chapter 4 
	TODO
		15.  The title of sect 4.4 would be better "Data Sources". It is not as important how you got the data  but what the source data consists of, i.e. is it monocular, does it have high res depth (laser, calibrated stereo), or low res depth (active camera & disparity map) 

		16. The results in 4.4.3 look good but you need to state the specific parameters you used with the FM2D and FM3D and ICP methods. Did you use the default values? What were they? Ie # scale factors used, gaussian window size, descriptor region radius etc.

		17. The results in 4.4.4 are a mixed bag, but that is good as well because it shows that with low res depth data the high noise levels means that you are going to get random performance for all methods which is fair enough

		18. In sec 4.4.5 there are not a lot of no comparative results it would be nicer to have something more.

		19. In table 4.2 it would be good to have a row at the bottom of each set that has the average mean square error across all the different noise values for each method to make it easier to get the picture. Where you have a fail just use and error of 20 or so.

		20. ditto for table 4.3

		21. You should make some comment in section 4.5 and 4.6 that since you are using the active camera data source that don't give the best results that it affects the performance margin of FVR compared to the FM and ICP methods

		22. Figure 4.27 shows reconstructions for FVR but none for ICP and FM you state that they are difficult to reconstruct with these, but a picture is worth 1000 words the worse they look the better it is for you. If you really can not reconstruct these then please given specific and detailed justification of why they are difficult to reconstruct.

		24. There is no run time comparison to verify your theoretical complexity in sect 3.1.4. Why not?
			Even if you can't give a full comparison against all methods at least give some indication of run-time in milliseconds for your implementation of FVR, FFVR etc. I think it is important to include some actual run times. 

		25. In sect 4.8 - 4.12 you should only present results that are novel and compare them against those in your honours thesis. 



