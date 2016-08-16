today:
	1. do pc
	2. capture or salvage test data [note it]
	3. test out algorithms
	4. do mode?
	
	do my methods
	do fm : surf & sift
	3d fm?

	algorithm implementation:
		
		icp  : v<R3>, Pix3D 
		fm   : Pix3D
		pc   : 
		mode : 
	1. setup my phd project:
		RGB-D data comes from my Pix3D class and corresponding files
		VMat/vector<R3> data can be computed from a variety of places
		VMat with Pix3D - used by the feature matching methods
		
	

to do:
	1. implement an evaluation system: 
		input:
			RGB-D data
			RGB data
			3D models
		output:
			reconstructed scene
			results:
				error (MSE, PSNR)
				qualitative
				speed / performance
				camera angle difference
		algorithms:
			feature matching (with Fundamental Matrix calculation)
			ICP
			PC & friends
			MODE (FM)
		testing environment:
			add noise
			skip more frames
			difference amounts of rote,scale,translation
	2. Thesis:
		experiments:
		Literature Review:
		Methodology:
		Intro:
		Conclusions:
		Finalize:
		