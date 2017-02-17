#pragma once

#include <string>
#include "../basics/VMatF.h"
#include "../basics/Pixel3DSet.h"
#include "../phd/experiments.h"
#include "../basics/locv3.h"
#include <functional>
#include <vector>
//code by luke: for octree, ILQV and Plane-Tree encoder for volumetric data

namespace ll_compression
{

    //helper functions
    VMat openDataVMat(std::string name, int index = 0);

    //object and base class for octrees enclosing volumetric data
    class VMatOctant
{
private:
    cv::Point3i         _corner;
    int             _size;
    VMatOctant *    _parent;
    VMatOctant *    _children[8];
    double          _mse;

public:
    VMatOctant(int defaultSize = 256);
    VMatOctant(const VMatOctant & input);
    VMatOctant & operator = (const VMatOctant & input);
    float getAverageColor(VMat & volume);
    virtual int countBits() = 0;
    double bytes();
    double kilobytes();
    double megabytes();
    ~VMatOctant();
    void forEach(std::function<void(VMatOctant *, int index, int level)> f);
    int height();
    int count();
    void freeChildren();
    static void getSubCubes(const cv::Point3i & currentCorner, const int & currentSize, std::vector<cv::Point3i> & corners, int & newSize);
    void reset(int size = 256);
    bool isLeaf() const;
    //setters and getters
    VMatOctant * getParent() const;
    void setParent(VMatOctant * parent);
    cv::Point3i getCorner() const;
    void setCorner(const cv::Point3i & corner);
    int getSize() const;
    void setSize(const int & size);
    VMatOctant * getChild(int index) const;
    void setChild(int index, VMatOctant * child);
    double getMse() const;
    void setMse(const double & mse);
    static double psnr(VMat & v1, VMat & v2);
};


//actual octree object inheriting from the base class:
class LOctV : public VMatOctant
{
private:
    unsigned char color;
public:
    LOctV();
    LOctV(int defaultSize);
    void computeRepresentation(VMat & volume);
    void computeMse(VMat & volume);
    void split(VMat & volume, double threshold, int minCube = 1);
    unsigned char getColor() const;
    void setColor(unsigned char color);
    int countBits();
    VMat out();
};



class ILQV : public VMatOctant
{
private:
    float colors[8];
public:
    ILQV();
    ILQV(int defaultSize);
    int getSubCornerIndex(int x, int y, int z, int hs);
    float interpolate(float a, float b, float t);
    float getColor(int x, int y, int z, cv::Point3i corner, int size);
    void computeRepresentation(VMat & volume);
    void computeMse(VMat & volume);
    void split(VMat & volume, double threshold, int minChild);
    int countBits();
    VMat out();
};

}
