today:
	1. setup my phd project:
		RGB-D data comes from my Pix3D class and corresponding files
		VMat data can be computed from a variety of places
		VMat with Pic - used by the feature matching methods
	2. setup my algorithm
	3. write the other algorithms
	4. setup test environment
	5. gather data
	6. do tests


todo:
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
		testing env:
			add noise
			skip more frames
			difference ammounts of rote,scale,trans
	2. Thesis:
		experiments:
		Literature Review:
		Methodology:
		Intro:
		Conclusions:
		Finalize:
		