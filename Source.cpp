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



class PlaneTreeV : public VMatOctant
{
private:
    float scalars[6];
public:
    PlaneTreeV() : VMatOctant()
    {
        for(int i = 0; i < 6; i++) scalars[i] = 0.0f;
    }

    PlaneTreeV(int defaultSize) : VMatOctant(defaultSize)
    {
        for(int i = 0; i < 6; i++) scalars[i] = 0.0f;
    }


    float getColor(int x, int y, int z, Point3i corner)
    {
        if(getSize() <= 4.0f) return scalars[0];
        float X = x - corner.x;
        float Y = y - corner.y;
        float Z = z - corner.z;
        return scalars[0]*X + scalars[1] + scalars[2]*Y + scalars[3] + scalars[4]*Z + scalars[5];
    }

    void computeRepresentationBasic(VMat & volume)
    {
        scalars[0] = getAverageColor(volume);
    }


    void computeRepresentation(VMat & volume)
    {
        Point3i corner = getCorner();
        int size = getSize();

        if(size <= 2)
        {
            computeRepresentationBasic(volume);
            return;
        }

        int s3 = size * size * size;
        Mat X = Mat::zeros(Size(6, s3), CV_32FC1);
        Mat Y = Mat::zeros(Size(1, s3), CV_32FC1);

        int _ = 0;

        for(int z = corner.z, _z = 0; z < corner.z + size; z++, _z++)
        {
            for(int y = corner.y, _y = 0; y < corner.y + size; y++, _y++)
            {
                for(int x = corner.x, _x = 0; x < corner.x + size; x++, _x++)
                {
                    float V = volume(x,y,z) * 255.0f;

                    Y.at<float>(_, 0) = V;

                    X.at<float>(_, 0) = (float)_x;
                    X.at<float>(_, 1) = 1.0f;
                    X.at<float>(_, 2) = (float)_y;
                    X.at<float>(_, 3) = 1.0f;
                    X.at<float>(_, 4) = (float)_z;
                    X.at<float>(_, 5) = 1.0f;
                    _++;
                }
            }
        }

        Mat B = least_squares(X, Y);
        for(int i = 0; i < 6; i++) scalars[i] = B.at<float>(i, 0);
    }



    void computeMse(VMat & volume)
    {
        Point3i corner = getCorner();
        int size = getSize();
        double sum = 0.0;
        int s3 = size * size * size;
        for(int z = corner.z; z < corner.z + size; z++)
        {
            for(int y = corner.y; y < corner.y + size; y++)
            {
                for(int x = corner.x; x < corner.x + size; x++)
                {
                    double difference = ((double)(volume(x,y,z) * 255.0f)) - getColor(x,y,z, corner);
                    sum += difference * difference;
                }
            }
        }
        setMse(sum / (double)s3);
    }

    void split(VMat & volume, double threshold, int minChild = 1)
    {
        computeRepresentation(volume);
        computeMse(volume);
        if(getMse() > threshold && getSize() > minChild) //split
        {
            vector<Point3i> corners; int newSize;
            VMatOctant::getSubCubes(getCorner(), getSize(), corners, newSize);
            for(int i = 0; i < 8; i++)
            {
                PlaneTreeV * child = new PlaneTreeV;
                child->setCorner(corners[i]);
                child->setSize(newSize);
                setChild(i, child);
                child->split(volume, threshold, minChild);
            }

        }
    }

    int countBits()
    {
        int numberOfBits = 0;
        stack<PlaneTreeV*> st;
        st.push(this);

        while(!st.empty())
        {
            PlaneTreeV* x = st.top(); st.pop();
            numberOfBits++; //0 is leaf, 1 is parent
            if(x->isLeaf())
            {
                if(x->getSize() > 4) numberOfBits += 6*4; //for the colors
                else numberOfBits += 4;
            }else
            {
                numberOfBits += 8; //for the children locations
                for(int i = 0; i < 8; i++)
                {
                    PlaneTreeV * c = (PlaneTreeV*)x->getChild(i);
                    if(c) st.push(c);
                }
            }

        }

        return numberOfBits;
    }


    VMat out()
    {
        VMat ret = getSize();

        stack<PlaneTreeV*> st;
        st.push(this);

        while(!st.empty())
        {
            PlaneTreeV * self = st.top(); st.pop();
            if(self->isLeaf())
            {
                //if it is a leaf: add it
                Point3i corner = self->getCorner();
                int size = self->getSize();
                for(int z = corner.z; z < corner.z + size; z++)
                    for(int y = corner.y; y < corner.y + size; y++)
                        for(int x = corner.x; x < corner.x + size; x++)
                        {
                            ret(x,y,z) = self->getColor(x,y,z, corner) / 255.0f;
                        }
            }else for(int i = 0; i < 8; i++) if(self->getChild(i)) st.push((PlaneTreeV*)self->getChild(i));
        }

        return ret;
    }


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
