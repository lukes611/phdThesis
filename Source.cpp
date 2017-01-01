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

using namespace std;
using namespace cv;
using namespace ll_R3;
using namespace ll_cam;
using namespace ll_measure;
using namespace ll_fmrsc;
using namespace ll_experiments;

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


VMat openDataVMat(string name, int index = 0)
{
    CapturePixel3DSet video = ll_experiments::openData(name, 1);
    Pix3D frame;
    video.read_frame(frame, index);

    Pixel3DSet pix3d = frame;
    VMat vframe(256, pix3d, 0.0f, true);

    return vframe;
}



class VMatOctant
{
private:
    Point3i         _corner;
    int             _size;
    VMatOctant *    _parent;
    VMatOctant *    _children[8];
    double          _mse;

public:
    VMatOctant(int defaultSize = 256)
    {
        //cout << "calling default constructor" << endl;
        reset(defaultSize);
    }
    VMatOctant(const VMatOctant & input)
    {
        setCorner(input.getCorner());
        setSize(input.getSize());
        setParent(input.getParent());
        for(int i = 0; i < 8; i++) setChild(i, input.getChild(i));
        setMse(input.getMse());
    }

    VMatOctant & operator = (const VMatOctant & input)
    {
        freeChildren();
        setCorner(input.getCorner());
        setSize(input.getSize());
        setParent(input.getParent());
        for(int i = 0; i < 8; i++) setChild(i, input.getChild(i));
        setMse(input.getMse());
        return *this;
    }

    float getAverageColor(VMat & volume)
    {
        Point3i corner = getCorner();
        int size = getSize();
        int sum = 0;
        int s3 = size * size * size;
        for(int z = corner.z; z < corner.z + size; z++)
        {
            for(int y = corner.y; y < corner.y + size; y++)
            {
                for(int x = corner.x; x < corner.x + size; x++)
                {
                    sum += (int)(volume(x,y,z) * 255.0f);
                }
            }
        }
        return ((unsigned char)(sum / (double)s3));
    }

    virtual int countBits() = 0;

    double bytes()
    {
        return countBits() / 8.0;
    }

    double kilobytes()
    {
        return bytes() / 1024.0;
    }

    double megabytes()
    {
        return kilobytes() / 1024.0;
    }





    ~VMatOctant()
    {
        //freeChildren();
    }

    void forEach(function<void(VMatOctant *, int index, int level)> f)
    {
        stack<tuple<VMatOctant*,int,int>> st;
        st.push(tuple<VMatOctant*,int,int>(this, 0,0));
        while(!st.empty())
        {
            //pop off stack
            tuple<VMatOctant*,int,int> item = st.top();
            st.pop();
            //do
            f(get<0>(item), get<1>(item), get<2>(item));
            //for each child: put on stack
            for(int i = 0; i < 8; i++)
            {
                VMatOctant * child = get<0>(item)->getChild(i);
                if(child)
                {
                    st.push(tuple<VMatOctant*,int,int>(child, i,get<2>(item) + 1));
                }
            }
        }
    }

    int height()
    {
        int h = 0;
        forEach([&h](VMatOctant* o, int index, int level)->void
        {
            h = h > level ? h : level;
        });
        return h;
    }

    int count()
    {
        int c = 0;
        forEach([&c](VMatOctant* o, int index, int level)->void
        {
            c++;
        });
        return c;
    }

    void freeChildren()
    {
        for(int i = 0; i < 8; i++)
        {
            VMatOctant * child = getChild(i);
            if(child != NULL)
            {
                child->freeChildren();
                delete child;
                setChild(i, NULL);
            }
        }
    }

    static void getSubCubes(const Point3i & currentCorner, const int & currentSize, vector<Point3i> & corners, int & newSize)
    {
        //newSize is half the old size
        newSize = currentSize / 2;

        //clean and add corners
        corners.clear();
        //original corner
        corners.push_back(Point3i(currentCorner.x, currentCorner.y, currentCorner.z));
        corners.push_back(Point3i(currentCorner.x + newSize, currentCorner.y, currentCorner.z));
        corners.push_back(Point3i(currentCorner.x + newSize, currentCorner.y + newSize, currentCorner.z));
        corners.push_back(Point3i(currentCorner.x, currentCorner.y + newSize, currentCorner.z));

        //add other corners which have a different z value
        for(int i = 0; i < 4; i++)
            corners.push_back(Point3i(corners[i].x, corners[i].y, corners[i].z + newSize));
    }



    void reset(int size = 256)
    {
        setSize(size);
        setParent(NULL);
        setCorner(Point3i(0,0,0));
        setMse(0.0);
        for(int i = 0; i < 8; i++) setChild(i, NULL);
    }

    bool isLeaf() const
    {
        for(int i = 0; i < 8; i++) if(getChild(i) != NULL) return false;
        return true;
    }

    /*void split()
    {
        int newSize;
        vector<Point3i> newCorners;
        VMatOctant::getSubCubes(getCorner(), getSize(), newCorners, newSize);
        for(int i = 0; i < 8; i++)
        {
            VMatOctant * child = new VMatOctant;
            child->setSize(newSize);
            child->setCorner(newCorners[i]);
            setChild(i, child);
        }
    }
    */
    //setters and getters
    VMatOctant * getParent() const { return _parent; }
    void setParent(VMatOctant * parent) { _parent = parent; }
    Point3i getCorner() const { return _corner; }
    void setCorner(const Point3i & corner) { _corner = corner; }
    int getSize() const { return _size; }
    void setSize(const int & size) { _size = size; }

    VMatOctant * getChild(int index) const { if(index >= 0 && index < 8) return _children[index]; return NULL; }
    void setChild(int index, VMatOctant * child) { if(index >= 0 && index < 8) _children[index] = child; }

    double getMse() const { return _mse; }
    void setMse(const double & mse) { _mse = mse; }

    static double psnr(VMat & v1, VMat & v2)
    {
        VMat a = v1;
        VMat b = v2;
        a *= 255.0f;
        b *= 255.0f;

        double mse = a.mse(b);

        return 10.0f * log10((255.0f * 255.0f) / mse);
    }
};

class LOctV : public VMatOctant
{
private:
    unsigned char color;
public:
    LOctV() : VMatOctant()
    {
        color = 0x00;
    }

    LOctV(int defaultSize) : VMatOctant(defaultSize)
    {
        color = 0x00;
    }



    void computeRepresentation(VMat & volume)
    {
        setColor(getAverageColor((volume)));
    }

    void computeMse(VMat & volume)
    {
        Point3i corner = getCorner();
        int size = getSize();
        int sum = 0;
        int s3 = size * size * size;
        for(int z = corner.z; z < corner.z + size; z++)
        {
            for(int y = corner.y; y < corner.y + size; y++)
            {
                for(int x = corner.x; x < corner.x + size; x++)
                {
                    int difference = ((int)(volume(x,y,z) * 255.0f)) - color;
                    sum += difference * difference;
                }
            }
        }
        setMse(sum / (double)s3);
    }

    void split(VMat & volume, double threshold, int minCube = 1)
    {
        computeRepresentation(volume);
        computeMse(volume);
        if(getMse() > threshold && getSize() > minCube) //split
        {
            vector<Point3i> corners; int newSize;
            VMatOctant::getSubCubes(getCorner(), getSize(), corners, newSize);
            for(int i = 0; i < 8; i++)
            {
                LOctV * child = new LOctV;
                child->setCorner(corners[i]);
                child->setSize(newSize);
                setChild(i, child);
                child->split(volume, threshold, minCube);
            }

        }else
        {
            //cout << "ending at" << getSize() << endl;
        }
    }

    unsigned char getColor() const { return color; }
    void setColor(unsigned char color) { this->color = color; }

    int countBits()
    {
        int numberOfBits = 0;
        stack<VMatOctant*> st;
        st.push(this);

        while(!st.empty())
        {
            VMatOctant* x = st.top(); st.pop();
            numberOfBits++; //0 is leaf, 1 is parent
            if(x->isLeaf())
            {
                numberOfBits += 8; //for the color
            }else
            {
                numberOfBits += 8; //for the children locations
                for(int i = 0; i < 8; i++)
                {
                    VMatOctant * c = x->getChild(i);
                    if(c) st.push(c);
                }
            }

        }

        return numberOfBits;
    }

    VMat out()
    {
        VMat ret = getSize();

        stack<LOctV*> st;
        st.push(this);

        while(!st.empty())
        {
            LOctV * self = st.top(); st.pop();
            if(self->isLeaf())
            {
                //if it is a leaf: add it
                Point3i corner = self->getCorner();
                int size = self->getSize();
                for(int z = corner.z; z < corner.z + size; z++)
                    for(int y = corner.y; y < corner.y + size; y++)
                        for(int x = corner.x; x < corner.x + size; x++)
                        {
                            ret(x,y,z) = self->getColor() / 255.0f;
                        }
            }else for(int i = 0; i < 8; i++) if(self->getChild(i)) st.push((LOctV*)self->getChild(i));
        }

        return ret;
    }
};

class ILQV : public VMatOctant
{
private:
    float colors[8];
public:
    ILQV() : VMatOctant()
    {
        for(int i = 0; i < 8; i++) colors[i] = 0.0f;
    }

    ILQV(int defaultSize) : VMatOctant(defaultSize)
    {
        for(int i = 0; i < 8; i++) colors[i] = 0.0f;
    }

    int getSubCornerIndex(int x, int y, int z, int hs)
    {
        int ret = 0;
        if(y / hs == 0)
        {
            if(x / hs == 0) ret = 0;
            else ret = 1;
        }else
        {
            if(x / hs == 0) ret = 3;
            else ret = 2;
        }
        if(z / hs > 0) ret += 4;
        return ret;
    }

    float interpolate(float a, float b, float t)
    {
        return a + (b-a) * t;
    }

    float getColor(int x, int y, int z, Point3i corner, int size)
    {
        float a = interpolate(colors[0], colors[3], (y-corner.y) / (float) size);
        float b = interpolate(colors[1], colors[2], (y-corner.y) / (float) size);

        float c1 = interpolate(a, b, (x-corner.x) / (float) size);

        float c = interpolate(colors[4], colors[7], (y-corner.y) / (float) size);
        float d = interpolate(colors[5], colors[6], (y-corner.y) / (float) size);

        float c2 = interpolate(c, d, (x-corner.x) / (float) size);

        return interpolate(c1, c2, (z-corner.z) / (float) size);


    }


    void computeRepresentation(VMat & volume)
    {

        Point3i corner = getCorner();
        int size = getSize();

        for(int i = 0; i < 8; i++) colors[i] = 0.0f;

        int hs = size / 2;
        int s3 = hs * hs * hs;
        float scalar = 1.0f / (float) s3;
        for(int z = corner.z, _z = 0; z < corner.z + size; z++, _z++)
        {
            for(int y = corner.y, _y = 0; y < corner.y + size; y++, _y++)
            {
                for(int x = corner.x, _x = 0; x < corner.x + size; x++, _x++)
                {
                    float V = volume(x,y,z) * 255.0f;
                    int index = getSubCornerIndex(_x, _y, _z, hs);
                    colors[index] += V * scalar;
                }
            }
        }
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
                    double difference = ((volume(x,y,z) * 255.0)) - getColor(x,y,z, corner, size);
                    sum += difference * difference;
                }
            }
        }
        setMse(sum / (double)s3);
    }

    void split(VMat & volume, double threshold, int minChild)
    {
        computeRepresentation(volume);
        computeMse(volume);
        //cout << "mse: " << getMse() << endl;
        //cout << "size: " << getSize() << endl;
        //cout << "i asked for " << threshold << endl;
        if(getMse() > threshold && getSize() > 2 && getSize() > minChild) //split
        {
            vector<Point3i> corners; int newSize;
            VMatOctant::getSubCubes(getCorner(), getSize(), corners, newSize);
            for(int i = 0; i < 8; i++)
            {
                ILQV * child = new ILQV(newSize);
                child->setCorner(corners[i]);
                child->setSize(newSize);
                child->split(volume, threshold, minChild);
                setChild(i, child);
            }

        }
    }

    int countBits()
    {
        int numberOfBits = 0;
        stack<ILQV*> st;
        st.push(this);

        while(!st.empty())
        {
            ILQV* x = st.top(); st.pop();
            numberOfBits++; //0 is leaf, 1 is parent
            if(x->isLeaf())
            {
                numberOfBits += 2*8; //for the colors
            }else
            {
                numberOfBits += 8; //for the children locations
                for(int i = 0; i < 8; i++)
                {
                    ILQV * c = (ILQV*)x->getChild(i);
                    if(c) st.push(c);
                }
            }

        }

        return numberOfBits;
    }


    VMat out()
    {
        VMat ret = getSize();

        stack<ILQV*> st;
        st.push(this);

        while(!st.empty())
        {
            ILQV * self = st.top(); st.pop();
            if(self->isLeaf())
            {
                //if it is a leaf: add it
                Point3i corner = self->getCorner();
                int size = self->getSize();
                for(int z = corner.z; z < corner.z + size; z++)
                    for(int y = corner.y; y < corner.y + size; y++)
                        for(int x = corner.x; x < corner.x + size; x++)
                        {
                            float oo = self->getColor(x,y,z, corner, size) / 255.0f;
                            oo *= 4.0f;
                            oo = floor(oo) / 4.0f;
                            //oo *= 255.0f;
                            ret(x,y,z) = oo;
                        }
            }else for(int i = 0; i < 8; i++) if(self->getChild(i)) st.push((ILQV*)self->getChild(i));
        }

        return ret;
    }


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

        if(size <= 4)
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
                if(x->getSize() > 4) numberOfBits += 6*8; //for the colors
                else numberOfBits += 8;
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


int main(int argc, char * * argv)
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
    int minCube = 8;

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
