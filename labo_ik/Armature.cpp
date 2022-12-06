#pragma once

#include "Armature.h"

using namespace gti320;

// Constructor
//
Link::Link(Link* _parent, const Vector3f& _eulerAng, const Vector3f& _trans) :
    parent(_parent), eulerAng(_eulerAng), trans(_trans)
{
    if (parent != nullptr)
    {
        parent->enfants.push_back(this);
    }
    M.setIdentity();
}

void Link::forward()
{
    // TODO Create a rotation matrix from the Euler angles
    //      of the current link.
    
    const auto rotMatrix = makeRotation(eulerAng(0), eulerAng(1), eulerAng(2));
        
    // TODO Create a local 4D rigid transformation matrix from the 
    //      3D rotation matrix and translation of the current link.

    Matrix4f rigidMatrix;
    rigidMatrix.setZero();
    rigidMatrix.block(0, 0, 3, 3) = rotMatrix;
    rigidMatrix(0, 3) = trans(0);
    rigidMatrix(1, 3) = trans(1);
    rigidMatrix(2, 3) = trans(2);
    rigidMatrix(3, 3) = 1;

    // TODO Update the global transformation for the link using the
    //      parent's rigid transformation matrix and the local transformation.
    //      Hint : the parent matrix should be post-multiplied.
    //      Hint : the root does not have a parent. You must consider this case.

    if (isRoot()) M = rigidMatrix;
    else M = parent->M * rigidMatrix;

    // TODO Update the transformation of child links
    // by recursion.
    if(!isEndEffector())
        enfants.at(0)->forward();
}

//NOUVELLE
//Very useful to build the jacobian matrix
//I need to do the cross product with the columns of the transform matrix
//block() returns a submatrix which is much harder to work with
Vector3f Link::getColumnOfMatrix(const int index) const {
    
    Vector3f globalCol;
    globalCol(0) = M(0, index);
    globalCol(1) = M(1, index);
    globalCol(2) = M(2, index);
    return globalCol;
}


Armature::Armature() : links(), root(nullptr)
{

}

Armature::~Armature()
{
    for (Link* l : links)
    {
        delete l;
    }
}

void Armature::updateKinematics()
{
    assert(root != nullptr);
    root->forward();
}

void Armature::pack(Vector<float, Dynamic>& theta)
{
    
    // TODO Collect the Euler angles of each link and put them
    //      into the dense vector @a theta
    assert(theta.size() == links.size() * 3);
    
    
    for (int i = 0; i < links.size(); ++i) {
        theta(i * 3) = links[i]->eulerAng(0);
        theta(i * 3 + 1) = links[i]->eulerAng(1);
        theta(i * 3 + 2) = links[i]->eulerAng(2);

    }

}

void Armature::unpack(const Vector<float, Dynamic>& theta)
{
    assert(theta.size() == 3 * links.size());

    for (int i = 0; i < links.size(); ++i) {
        links[i]->eulerAng(0) = theta(i * 3);
        links[i]->eulerAng(1) = theta(i * 3 + 1);
        links[i]->eulerAng(2) = theta(i * 3 + 2);
    }

    // TODO Extract the Euler angles contained in the 
    //      dense vector @a theta and update the angles
    //      for each link in the armature.
    // 

}
