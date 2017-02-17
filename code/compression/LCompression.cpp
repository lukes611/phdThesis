#include "LCompression.h"

#include <stack>
using namespace std;
using namespace cv;

namespace ll_compression
{
    //opens some 3d reconstruction frame as a VMat object
    VMat openDataVMat(std::string name, int index)
    {
        ll_pix3d::CapturePixel3DSet video = ll_experiments::openData(name, 1);
        Pix3D frame;
        video.read_frame(frame, index);

        Pixel3DSet pix3d = frame;
        VMat vframe(256, pix3d, 0.0f, true);

        return vframe;
    }

    //methods for Octree object: VMatOctant
    VMatOctant::VMatOctant(int defaultSize) { reset(defaultSize); }
    VMatOctant::VMatOctant(const VMatOctant & input)
    {
        setCorner(input.getCorner());
        setSize(input.getSize());
        setParent(input.getParent());
        for(int i = 0; i < 8; i++) setChild(i, input.getChild(i));
        setMse(input.getMse());
    }
    VMatOctant & VMatOctant::operator = (const VMatOctant & input)
    {
        freeChildren();
        setCorner(input.getCorner());
        setSize(input.getSize());
        setParent(input.getParent());
        for(int i = 0; i < 8; i++) setChild(i, input.getChild(i));
        setMse(input.getMse());
        return *this;
    }
    float VMatOctant::getAverageColor(VMat & volume)
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
                    if(volume.inbounds(x,y,z))
                        sum += (int)(volume(x,y,z) * 255.0f);
                    else s3--;
                }
            }
        }
        return ((sum / (double)s3));
    }
    double VMatOctant::bytes()
    {
        return countBits() / 8.0;
    }
    double VMatOctant::kilobytes()
    {
        return bytes() / 1024.0;
    }
    double VMatOctant::megabytes()
    {
        return kilobytes() / 1024.0;
    }
    VMatOctant::~VMatOctant()
    {
        //freeChildren();
    }
    void VMatOctant::forEach(function<void(VMatOctant *, int index, int level)> f)
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
    int VMatOctant::height()
    {
        int h = 0;
        forEach([&h](VMatOctant* o, int index, int level)->void
        {
            h = h > level ? h : level;
        });
        return h;
    }
    int VMatOctant::count()
    {
        int c = 0;
        forEach([&c](VMatOctant* o, int index, int level)->void
        {
            c++;
        });
        return c;
    }
    void VMatOctant::freeChildren()
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
    void VMatOctant::getSubCubes(const Point3i & currentCorner, const int & currentSize, vector<Point3i> & corners, int & newSize)
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
    void VMatOctant::reset(int size)
    {
        setSize(size);
        setParent(NULL);
        setCorner(Point3i(0,0,0));
        setMse(0.0);
        for(int i = 0; i < 8; i++) setChild(i, NULL);
    }

    bool VMatOctant::isLeaf() const
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
    VMatOctant * VMatOctant::getParent() const { return _parent; }
    void VMatOctant::setParent(VMatOctant * parent) { _parent = parent; }
    Point3i VMatOctant::getCorner() const { return _corner; }
    void VMatOctant::setCorner(const Point3i & corner) { _corner = corner; }
    int VMatOctant::getSize() const { return _size; }
    void VMatOctant::setSize(const int & size) { _size = size; }
    VMatOctant * VMatOctant::getChild(int index) const { if(index >= 0 && index < 8) return _children[index]; return NULL; }
    void VMatOctant::setChild(int index, VMatOctant * child) { if(index >= 0 && index < 8) _children[index] = child; }
    double VMatOctant::getMse() const { return _mse; }
    void VMatOctant::setMse(const double & mse) { _mse = mse; }
    double VMatOctant::psnr(VMat & v1, VMat & v2)
    {
        VMat a = v1;
        VMat b = v2;
        a *= 255.0f;
        b *= 255.0f;

        double mse = a.mse(b);

        return 10.0f * log10((255.0f * 255.0f) / mse);
    }


    //regular octree object for comrpession
    LOctV::LOctV() : VMatOctant()
    {
        color = 0x00;
    }
    LOctV::LOctV(int defaultSize) : VMatOctant(defaultSize)
    {
        color = 0x00;
    }
    void LOctV::computeRepresentation(VMat & volume)
    {
        setColor(getAverageColor((volume)));
    }
    void LOctV::computeMse(VMat & volume)
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
    void LOctV::split(VMat & volume, double threshold, int minCube)
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
    unsigned char LOctV::getColor() const { return color; }
    void LOctV::setColor(unsigned char color) { this->color = color; }
    int LOctV::countBits()
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
    VMat LOctV::out()
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


    //liqv
    ILQV::ILQV() : VMatOctant()
    {
        for(int i = 0; i < 8; i++) colors[i] = 0.0f;
    }
    ILQV::ILQV(int defaultSize) : VMatOctant(defaultSize)
    {
        for(int i = 0; i < 8; i++) colors[i] = 0.0f;
    }
    int ILQV::getSubCornerIndex(int x, int y, int z, int hs)
    {
        int ret = 0;
        if(y < hs)
        {
            if(x < hs) ret = 0;
            else ret = 1;
        }else
        {
            if(x < hs) ret = 3;
            else ret = 2;
        }
        if(z >= hs) ret += 4;
        return ret;
    }
    float ILQV::interpolate(float a, float b, float t)
    {
        return a + (b-a) * t;
    }
    float ILQV::getColor(int x, int y, int z, Point3i corner, int size)
    {


        float a = interpolate(colors[0], colors[3], (y-corner.y) / (float) size);
        float b = interpolate(colors[1], colors[2], (y-corner.y) / (float) size);

        float c1 = interpolate(a, b, (x-corner.x) / (float) size);

        float c = interpolate(colors[4], colors[7], (y-corner.y) / (float) size);
        float d = interpolate(colors[5], colors[6], (y-corner.y) / (float) size);

        float c2 = interpolate(c, d, (x-corner.x) / (float) size);

        return interpolate(c1, c2, (z-corner.z) / (float) size);


    }
    void ILQV::computeRepresentation(VMat & volume)
    {
        Point3i corner = getCorner();
        int size = getSize();





        for(int i = 0; i < 8; i++) colors[i] = 0.0f;

        //int hs = size / 2;
        int newSize;
        vector<Point3i> corners;
        VMatOctant::getSubCubes(corner, size, corners, newSize);

        for(int i = 0; i < corners.size(); i++)
        {
            ILQV tmp;
            tmp.setSize(newSize);
            tmp.setCorner(corners[i] - Point3i(newSize/2, newSize/2, newSize/2));
            colors[i] = tmp.getAverageColor(volume);
        }

        /*int s3 = hs * hs * hs;
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
        }*/
    }
    void ILQV::computeMse(VMat & volume)
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
    void ILQV::split(VMat & volume, double threshold, int minChild)
    {
        computeRepresentation(volume);
        computeMse(volume);
        //cout << "mse: " << getMse() << endl;
        //cout << "size: " << getSize() << endl;
        //cout << "i asked for " << threshold << endl;
        if(getMse() > threshold && getSize() > minChild && getSize() > 2) //split
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
    int ILQV::countBits()
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
    VMat ILQV::out()
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
                            //oo *= 4.0f;
                            //oo = floor(oo) / 4.0f;
                            //oo *= 255.0f;
                            ret(x,y,z) = oo;
                        }
            }else for(int i = 0; i < 8; i++) if(self->getChild(i)) st.push((ILQV*)self->getChild(i));
        }

        return ret;
    }


    //plane-tree volumetric edition
    PlaneTreeV::PlaneTreeV() : VMatOctant()
    {
        for(int i = 0; i < 6; i++) scalars[i] = 0.0f;
    }
    PlaneTreeV::PlaneTreeV(int defaultSize) : VMatOctant(defaultSize)
    {
        for(int i = 0; i < 6; i++) scalars[i] = 0.0f;
    }
    float PlaneTreeV::getColor(int x, int y, int z, Point3i corner)
    {
        if(getSize() <= 4.0f) return scalars[0];
        float X = x - corner.x;
        float Y = y - corner.y;
        float Z = z - corner.z;
        return scalars[0]*X + scalars[1] + scalars[2]*Y + scalars[3] + scalars[4]*Z + scalars[5];
    }
    void PlaneTreeV::computeRepresentationBasic(VMat & volume)
    {
        scalars[0] = getAverageColor(volume);
    }
    void PlaneTreeV::computeRepresentation(VMat & volume)
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
    void PlaneTreeV::computeMse(VMat & volume)
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
    void PlaneTreeV::split(VMat & volume, double threshold, int minChild)
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
    int PlaneTreeV::countBits()
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
    VMat PlaneTreeV::out()
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
}
