today:
	
	1. capture or salvage test data [note it]
	2. test out algorithms
	3. setup experiments
	
algorithm implementation:
	
	icp  	: Pix3D 
	fm   	: Pix3D
	pc   	: Pix3D
	pca_pc  : Pix3D
		
	

to do:
	1. implement an evaluation system: 
		input:
			RGB-D data
			RGB data
			3D models
		output:
			reconstructed scene
			results:
				error (MSE)
				qualitative
				speed / performance
				camera angle difference
		algorithms:
			feature matching w ransac
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
		