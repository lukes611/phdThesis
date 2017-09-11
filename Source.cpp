#include "code\phd\LPhDHelper.h"

Mat getColorIm(int n){
	stringstream ss;
	ss << string(LCPPDATA_DIR) << "/outfolder/color/" << n << ".png";
	Mat ret = imread(ss.str());
	return ret.clone();
}

Mat getDepthIm(int n){
	stringstream ss;
	ss << string(LCPPDATA_DIR) << "/outfolder/depth/" << n << ".png";
	Mat ret = imread(ss.str());
	return ret.clone();
}

unsigned char getMedian(vector<unsigned char> l)
{
	if(l.size() == 0) return 0.0f;
	sort(l.begin(), l.end());
	int mI = l.size() / 2;
	return l[mI];
}

unsigned char getMedian2(vector<unsigned char> l)
{

	int avg = 0;
	if(l.size() == 0) return 0;
	for(int i = 0; i < l.size(); i++) avg += l[i];
	return (unsigned char)((avg / (float)l.size()) + 0.5f);
}


void fixHole(unsigned char histogram[255], int i, bool * missing)
{
	bool hasPrev = false, hasNext = false;
	int prevVal = 0, nextVal = 0;
	for(int j = i-1; j > 0; j--)
	{
		if(missing[j])
		{
			hasPrev = true;
			prevVal = histogram[j];
		}
	}
	for(int j = i+1; j < 255; j++)
	{
		if(missing[j])
		{
			hasNext = true;
			nextVal = histogram[j];
		}
	}
	if(!hasNext && !hasPrev) return;
	if(!hasNext && hasPrev)
	{
		histogram[i] = prevVal;
		missing[i] = false;
		return;
	}
	if(hasNext && !hasPrev)
	{
		histogram[i] = nextVal;
		missing[i] = false;
		return;
	}
	int mn = nextVal < prevVal ? nextVal : prevVal;
	int mx = nextVal < prevVal ? prevVal : nextVal;
	histogram[i] = (mx-mn)/2 + mn;
	missing[i] = false;
}

Mat getDepthIm(int n, Mat m){
	stringstream ss;
	ss << string(LCPPDATA_DIR) << "/outfolder/depth/" << n << ".png";
	Mat ret = imread(ss.str(), CV_LOAD_IMAGE_GRAYSCALE);
	int Y = m.size().height;
	int X = m.size().width;
	Vec3b red(0, 0, 255);

	vector<unsigned char> histogram_[255];
	unsigned char histogram[255];
	bool missing[256];

	kitti::KittiPix3dSet ki = kitti::open("2011_09_26_drive_0001_sync", n);

	Mat dm = ki.getDepthMap();
	//dm *= 4;
	ll_32F1_to_UCF1(dm);
	
	
	Mat vdi = ki.validDepthImage.clone();
	//cout << ki.points.size() << " : " << (vdi.size().width * vdi.size().height) << endl;
	//imshow("vdi", dm);
	


	for(int y = 0; y < Y; y++)
	{
		for(int x = 0; x < X; x++)
		{
			if(m.at<Vec3b>(y,x) == red)
				ret.at<unsigned char>(y,x) = 0;
			if(ret.at<unsigned char>(y,x) > 0)ret.at<unsigned char>(y,x) = 255 - ret.at<unsigned char>(y,x);
			if(vdi.at<unsigned char>(y,x) > 0) //include this pixel in the mapping computation
			{
				histogram_[ret.at<unsigned char>(y,x)].push_back(dm.at<unsigned char>(y,x));
			}
		}
	}
	for(int i = 0; i < 255; i++)
	{
		histogram[i] = getMedian(histogram_[i]);
		missing[i] = histogram_[i].size() == 0;
		
	}
	for(int i = 0; i < 255; i++)
	{
		if(missing[i])
		{
			fixHole(histogram, i, missing);
		}
		//cout << i << " => " << (int)histogram[i] << endl;
	}

	for(int y = 0; y < Y; y++)
	{
		for(int x = 0; x < X; x++)
		{
			if(ret.at<unsigned char>(y,x) > 0)
			{
				ret.at<unsigned char>(y,x) = (unsigned char)(.5f * histogram[ret.at<unsigned char>(y,x)] + 0.5f * ret.at<unsigned char>(y,x));
			}
		}
	}

	return ret.clone();
}

Mat getMaskIm(){
	stringstream ss;
	ss << string(LCPPDATA_DIR) << "/outfolder/mask.png";
	Mat ret = imread(ss.str());
	return ret.clone();
}

//get depth - mask it, then read in the same depth map truth:
Pixel3DSet project(Mat ci, Mat dm)
{
	Pixel3DSet r;
	

	float scalar = 0.001f;
	int Y = ci.size().height;
	int X = ci.size().width;
	int hx = X/2;
	int hy = Y/2;
	for(int y = 0; y < Y; y++)
	{
		for(int x = 0; x < X; x++)
		{
			unsigned char d = dm.at<unsigned char>(y,x);
			if(d == 0) continue;
			float D = d/255.0f * 3000;
			R3 p((x - hx) * D * scalar, (y-hy) * D * scalar, D);
			Vec3b c = ci.at<Vec3b>(y,x);
			p.y = -p.y;
			r.push_back(p, c);
		}
	}
	cout << r.size() << endl;
	R3 mn,mx;
	r.min_max_R3(mn, mx);
	cout << mn << " / " << mx << endl;
	return r;
}

kitti::KittiPix3dSet monoFrame(int i, Mat * _mask = NULL)
{
	Mat mask;
	if(_mask == NULL) mask = getMaskIm();
	else mask = *_mask;
	kitti::KittiPix3dSet ret;
	ret.colorImage = getColorIm(i);
	ret.validDepthImage = Mat::zeros(ret.colorImage.size(), CV_8UC1);
	ret.points = vector<R3>(ret.colorImage.size().width * ret.colorImage.size().height);

	Mat d = getDepthIm(i, mask);

	float scalar = 0.001f;
	int Y = ret.colorImage.size().height;
	int X = ret.colorImage.size().width;
	int hx = X/2;
	int hy = Y/2;
	for(int y = 0; y < Y; y++)
	{
		for(int x = 0; x < X; x++)
		{
			unsigned char de = d.at<unsigned char>(y,x);
			if(de == 0) continue;
			float D = de/255.0f * 3000;
			R3 p((x - hx) * D * scalar, (y-hy) * D * scalar, D);
			//Vec3b co = ret.colorImage.at<Vec3b>(y,x);
			p.y = -p.y;
			//r.push_back(p, co);
			ret.validDepthImage.at<unsigned char>(y,x) = 0xFF;
			ret.points[y * X + x] = p;
		}
	}

	return ret;
}

void quantitativeExperimentKittiMono(string algorithm_name, int from = 0)
{
    kitti::KittiPix3dSet frame1, frame2;
	Pixel3DSet a, b;
	Mat mask = getMaskIm();

    frame1 = monoFrame(from, &mask);

	for (int _i = from+1; _i < 107; _i++)
	{
		int currentIndex = _i;
		//get match _m from i+1 to i
		double seconds = 0.0;
		double hde;
		int iters = 0;
		Pixel3DSet _;


		cout << algorithm_name << ", completed " << _i << " of " << 107 << endl;

		frame2 = monoFrame(currentIndex, &mask);


		a = frame1.getPoints(); b = frame2.getPoints();

		Mat _m = Mat::eye(Size(4, 4), CV_32FC1);

		//algorithms here:
		if (algorithm_name == "none") {
            //do-nothing
		}
#if defined(HASCUDA) || defined(HASFFTW)
		else if (algorithm_name == "FVR") {
			_m = ll_pc::pc_register(b, a, seconds);
		}

		else if (algorithm_name == "FVR3D") {
			_m = ll_pc::pc_register_pca_i(b, a, seconds);
		}
		else if (algorithm_name == "FVR3D-2") {
			_m = ll_pc::pc_pca_icp(b, a, seconds);
		}else if (algorithm_name == "FFVR"){
            _m = ll_pc::ffvr(b, a, seconds);
		}
#endif
        else if(algorithm_name == "PCA"){
            _m = ll_pca::register_pca(b, a, seconds);
        }else if (algorithm_name == "ICP") {
			_m = Licp::icp(b, a, _, hde, seconds, iters);
		}
		else if (algorithm_name == "FM2D") {
			ll_fmrsc::registerPix3D("surf", frame2, frame1, _m, seconds, true, -1);
		}else if(algorithm_name == "FM3D"){
		    _m = LukeLincoln::sift3DRegister(b, a, seconds, true, 256);
		}



		//:end

		//compute the errors
		b.transform_set(_m);
		//hde = ll_measure::hausdorff(a, b);
		double msee, pme;
		hde = 21.0;
		ll_measure::error_metrics(a, b, hde, msee, pme);

		//void saveV20(string data_name, string alg_name, int frame1, int frame2, float seconds, float hd)
		saveKitti("kitti.mono_experiments", algorithm_name, _i, _i-1, seconds, hde);


		frame1 = frame2;
	}




}


int main()
{
		//quantitativeExperimentKittiMono("none");
		//quantitativeExperimentKittiMono("FM2D");
		//quantitativeExperimentKittiMono("FM3D", 39);
		//quantitativeExperimentKittiMono("ICP");
		//quantitativeExperimentKittiMono("PCA", 78);
		//quantitativeExperimentKittiMono("FVR3D");
		//quantitativeExperimentKittiMono("FVR3D-2");
		//quantitativeExperimentKittiMono("FVR", 12);
		quantitativeExperimentKittiMono("FFVR");
	return 0;
}

int mainTestMono()
{
	Mat m = getMaskIm();
	cout << "working on monoexperiments" << endl;
	int i = 0;
	for(i = 0; i < 107; i++)
	{
		kitti::KittiPix3dSet km = monoFrame(i, &m);
		//kitti::KittiPix3dSet ki = kitti::open("2011_09_26_drive_0001_sync", i);
		//Mat c = getColorIm(i);
		//Mat d = getDepthIm(i, m);
		imshow("color", km.colorImage);
		//imshow("depth", km.getDepthMap());
		//imshow("td", ki.getDepthMap());
		//waitKey();
		//Pixel3DSet px = project(c, d);
		Pixel3DSet px = km.getPoints();
		LLPointers::setPtr("object", &px);
		viewPixel3DSet();
	}
	

	return 0;
}






int mainBasic()
{
	//currate("rotation.10deginc.office1", 0, 1, "10deg");
	string tsets[1] = {
		"trans.5cminc.office3"
	};
	string rsets[3] = {
		"rotation.10deginc.office3",
		"rotation.10deginc.office4",
		"rotation.10deginc.office7"

	};
	string algorithms[9] = {
        "FM2D",
        "FVR",
		"none",
		"ICP",
		"FVR3D",
		"FVR3D-2",
		"FFVR",
		"PCA",
		"FM3D"

	};

	//for (int i = 0; i < 9; i++)
		lcamExpR(rsets[2], algorithms[1]);

    //saveReg(rsets[2], "FVR", 0, 3);
    //saveReg(rsets[1], "FVR", 0, 2);

	return 0;
}



int mainOld(int argc, char * * argv)
{

	string namesList[20] = {

        "Apartment.Texture.rotate", //0
        "Apartment.Texture.rotateXAxis", //1
        "Boxes.Texture.arbitrarycamera", // 2
        "Boxes.Texture.rotate", // 3
        "Boxes.Texture.zoomOut", //4
        "Desk.Texture.Translation", //5
        "IndoorSpace.tc.translation", //6
        "Kitchen.littleTexture.pan", //7
        "Kitchen.littleTexture.zoom", //8
        "OfficeDesk.Texture.rotationLift", //9
        "Office.Texture.blindSpotRotation", //10
        "Office.TexturedItems.Translation", //11
        "Office.Texture.rotation", //12
        "Office.Texture.rotationXAxis", //13
        "Office.Texture.Translation", //14
        "Outside.NoTexture.rotation", //15
        "Outside.NoTexture.translation", //16
        "Outside.TextureConfusion.rotation", //17
        "Outside.TextureConfusion.Translation", //18
        "PlantsOutdoors.tc.rotation" //19
    };

    //string fn = "PlantsOutdoors.tc.rotation";



	//save images: 4 per video file
	/*for(int videoI = 0; videoI < 20; videoI++)
	{
		CapturePixel3DSet video = openData(namesList[videoI], 1);
		for(int i = 0; i < 4; i++)
		{
			int j = 4 + i * 6;
			Pix3D frame;
			video.read_frame(frame, j);
			Mat image;
			frame.colorImage(image);
			//imshow("image", image);
			//waitKey(100);
			stringstream file_name;
			file_name << "C:/Users/luke/Documents/Visual Studio 2012/Projects/PhD 16 Sem2/PhD 16 Sem2/thesis/images/experiments/"
				<< "test_data/" << namesList[videoI] << "." << i << ".png";
			cv::resize(image, image, Size(640/2, 480/2));
			cv::imwrite(file_name.str(), image);

			//cout << file_name.str() << endl;
		}
	}*/

	//testSetPix3d(namesList[0]);
	//test("Apartment.Texture.rotate", Point3d(5.0f, 2.0f, 0.0f), 1.0f, Point3d(0.0, 1.0, 8.0));

	//for(int i = 13; i < 19; i++)


    /*
	to-do:
		2. create experiments 3.0 []
            feature matching 2d []
            feature matching 3d []
            fvr []
            fvr3d []
            fvr3d-2 []
            ffvr []
            icp []
            PCA []

		3. run experiments 3.0
		4. setup the other test experiments
		5. add to the thesis
		6. finalize thesis experiments section
		7. re-write the section also
			add the r-table required
		8. re-write intro.conclusion.methodology (somewhat)


	*/

	string namesList2[5] = {
		"2011_09_26_drive_0001_sync",
		"2011_09_26_drive_0002_sync",
		"2011_09_26_drive_0005_sync",
		"2011_09_26_drive_0091_sync",
		"2011_09_26_drive_0095_sync"
	};
	int countList[5] = {107, 76, 153, 339, 267};

	//for (int i = 2; i < 5; i++)
	{
		int i = 4;
		string kittiData = namesList2[i];
		vector<int> inds = ll_experiments::rng(252, countList[i], 1);
		//vector<int> inds2 = ll_experiments::rng(99, countList[i], 1);
		//quantitativeExperimentKitti10("none", kittiData, inds);
		//quantitativeExperimentKitti10("FM2D", kittiData, inds2);
		//quantitativeExperimentKitti10("FM3D", kittiData, inds);
		//quantitativeExperimentKitti10("ICP", kittiData, inds);
		//quantitativeExperimentKitti10("PCA", kittiData, inds);
		//
		//quantitativeExperimentKitti10("FVR3D", kittiData, inds);
		//quantitativeExperimentKitti10("FVR3D-2", kittiData, inds);
		quantitativeExperimentKitti10("FVR", kittiData, inds);
		//quantitativeExperimentKitti10("FFVR", kittiData, inds);
	}

	if(0){
		string fn = namesList[1];
		//exp1("Apartment.Texture.rotate", ll_experiments::rng(15, 20, 1));
		int start = 20/*4*/, to = 30, inc = 1;
		vector<int> inds = ll_experiments::rng(start, to, inc);
		//quantitativeExperiment20("none", fn, inds);
		//quantitativeExperiment20("FM2D", fn, inds);
		//quantitativeExperiment20("FM3D", fn, inds);
		//quantitativeExperiment20("ICP", fn, inds);
		//quantitativeExperiment20("PCA", fn, inds);
		
		//quantitativeExperiment20("FVR3D", fn, inds);
		quantitativeExperiment20("FVR3D-2", fn, inds);
		//quantitativeExperiment20("FVR", fn, inds);
		
		//quantitativeExperiment20("FFVR", fn, inds);


	}



	return 0;
}


int mainmanual(int argc, char * * argv)
{
    return 1;
	//usage: experiment-type algorithm  dataset index
	bool usedProperly = true;
	string algorithm, experimentType, dataset;
	int index = 0;
	if (argc != 5) usedProperly = false;
	if (usedProperly)
	{
		if (sscanf(argv[4], "%d", &index) != 1) usedProperly = false;
		if (usedProperly)
		{
			algorithm = argv[2];
			experimentType = argv[1];
			dataset = argv[3];

			if (experimentType != "2.0" && experimentType != "kitti") usedProperly = false;
			if (algorithm != "none"&&algorithm != "FM2D"&&algorithm != "FM3D"&&algorithm != "ICP"&&algorithm != "PCA"
				&&algorithm != "FVR"&&algorithm != "FVR3D"&&algorithm != "FVR3D-2"&&algorithm != "FFVR")
				usedProperly = false;
		}
	}


	if (usedProperly)
	{
		cout << "OK" << endl;
		cout << "type: " << experimentType << endl;
		cout << "algorithm: " << algorithm << endl;
		cout << "dataset: " << dataset << endl;
		cout << "index: " << index << endl;
		if (experimentType == "2.0")
		{
			vector<int> inds = ll_experiments::rng(index, index+2, 1);
			quantitativeExperiment20(algorithm, dataset, inds);
		}
		else if (experimentType == "kitti")
		{
			vector<int> inds = ll_experiments::rng(index, index + 2, 1);
			quantitativeExperimentKitti10(algorithm, dataset, inds);
		}

	}
	else
	{
		cout << "ERROR" << endl;
		/*index = 84;
		vector<int> inds = ll_experiments::rng(index, index+2, 1);
		for (int i = 0; i < inds.size(); i++) cout << inds[i] << endl;
		quantitativeExperiment20("none", "Apartment.Texture.rotate", inds);*/
	}
}
