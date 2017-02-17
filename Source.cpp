#include <iostream>
#include <string>
#include <vector>
#include <stack>
#include "code/basics/locv3.h"
#include "code/phd/experiments.h"
#include "code/basics/R3.h"
#include "code/basics/llCamera.h"
#include "code/basics/VMatF.h"
#include "code/basics/LTimer.h"
#include "code/phd/Licp.h"
#include "code/phd/measurements.h"
#include "code/phd/fmRansac.h"
#include "code/pc/TheVolumePhaseCorrelator.h"
#include "code/phd/Lpcr.h"
#include "code/script/LScript.h"
#include "code/phd/LSift.h"
#include "code/basics/BitReaderWriter.h"
#include "code/compression/LCompression.h"

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;

using namespace ll_compression;

string namesList[20] = {
    "Apartment.Texture.rotate",
    "Apartment.Texture.rotateXAxis",
    "Boxes.Texture.arbitrarycamera",
    "Boxes.Texture.rotate",
    "Boxes.Texture.zoomOut",
    "Desk.Texture.Translation",
    "IndoorSpace.tc.translation",
    "Kitchen.littleTexture.pan",
    "Kitchen.littleTexture.zoom",
    "OfficeDesk.Texture.rotationLift",
    //"office.move1cm",
    "Office.Texture.blindSpotRotation",
    "Office.TexturedItems.Translation",
    "Office.Texture.rotation",
    "Office.Texture.rotationXAxis",
    "Office.Texture.Translation",
    "Outside.NoTexture.rotation",
    "Outside.NoTexture.translation",
    "Outside.TextureConfusion.rotation",
    "Outside.TextureConfusion.Translation",
    "PlantsOutdoors.tc.rotation"
};





int main(){
    //read in bunny
    //put into volume
    //view volume
    return 0;
}

int main2(int argc, char * * argv)
{

    //todo:
    /*
        read in obj [done]
        build octree for volumes [done]
        build st for volumes
        test differences at different psnrs
        put in excel
        put in graphs
    */
    VMat obj = openDataVMat(namesList[0], 0);

    //5, 10, 20
    vector<double> thresholds;
    thresholds.push_back(20.0f);
    thresholds.push_back(10.0f);
    thresholds.push_back(7.0f);
    thresholds.push_back(5.0f);
    thresholds.push_back(3.0f);
    thresholds.push_back(2.0f);
    thresholds.push_back(1.0f);
    thresholds.push_back(0.9f);
    thresholds.push_back(0.8f);
    thresholds.push_back(0.6f);
    thresholds.push_back(0.5f);
    thresholds.push_back(0.4f);
    thresholds.push_back(0.2f);
    thresholds.push_back(0.1f);
    thresholds.push_back(0.08f);
    thresholds.push_back(0.05f);
    thresholds.push_back(0.02f);
    thresholds.push_back(0.01f);

    string type = "pt";
    int minCube = 1;

    for(int _j = 0, minCube = 1; _j < 4; _j++, minCube *= 2)
    for(int i = 0; i < thresholds.size(); i++)
    {
        double threshold = thresholds[i];
        VMat input = obj;
        VMat output;
        int numBits = 0;
        double psnr = 0.0;

        if(type == "ot")
        {
            LOctV _ = 256;
            _.split(input, threshold, minCube);
            output = _.out();
            psnr = _.psnr(input, output);
            numBits = _.countBits();
        }else if(type == "ilqv")
        {
            ILQV _ = 256;
            _.split(input, threshold, minCube);
            output = _.out();
            psnr = _.psnr(input, output);
            numBits = _.countBits();
        }else if(type == "pt")
        {
            PlaneTreeV _ = 256;
            _.split(input, threshold, minCube);
            output = _.out();
            psnr = _.psnr(input, output);
            numBits = _.countBits();
        }

        cout << threshold << endl;

        cout << type << ": " << psnr << ", " << numBits << endl;
        cout << "\n******\n";

        stringstream outputFileName;
        outputFileName << EXPS_DIR << "/compression/" << type << ".csv";
        stringstream header;
        header << "type,No. bits,psnr";
        stringstream data;
        data << type << "," << numBits << "," << psnr;
        ll_experiments::appendData(outputFileName.str(), header.str(), data.str());


    }

	return 0;
}
