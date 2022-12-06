#pragma once

#include "IKSolver.h"
#include "Armature.h"
#include "SVD.h"

using namespace gti320;

namespace
{
}

IKSolver::IKSolver(Armature* _armature, Vector3f& _targetPos) : m_armature(_armature), m_targetPos(_targetPos), m_J()
{
}


float IKSolver::getError(Vector3f& dx) const
{
    // TODO Compute the error between the current end effector 
    //      position and the target position 
    dx = m_targetPos - m_armature->getEndAffector()->getColumnOfMatrix(3);
   

    return dx.norm();
}

void IKSolver::solve()
{
    const int numLinks = m_armature->links.size();
    const int dim = 3 * (numLinks);
    m_J.resize(3, dim);

    // We assume that the last link is the "end effector"
    //
    Vector3f endGlobalPos = m_armature->getEndAffector()->getColumnOfMatrix(3);
   
    // TODO Build the Jacobian matrix m_J.
    //      Each column corresponds to a separate 
    

    for (int i = 0; i < numLinks; ++i) {

        Vector3f r = endGlobalPos - m_armature->links[i]->getColumnOfMatrix(3);

        for (int j = 0; j < 3; ++j) {
            Vector3f w = m_armature->links[i]->getColumnOfMatrix(j);
            Vector3f cross = crossProduct(w, r);
            m_J(0, i * 3 + j) = cross(0);
            m_J(1, i * 3 + j) = cross(1);
            m_J(2, i * 3 + j) = cross(2);
        }       
    }

    // TODO Compute the error between the current end effector 
    //      position and the target position by calling getError()
    // 

    Vector3f dx;
    float error = getError(dx);

    // TODO Compute the change in the joint angles by solving:
    //    df/dtheta * delta_theta = delta_x
    //  where df/dtheta is the Jacobian matrix.
    //    
    //
    SVD t(m_J);
    t.decompose();

    int const size = t.getSigma().size();
    Vector<float> z(size);
    z.setZero();

    auto const uTb = t.getU().transpose<float,-1 ,3, 0>() * dx;
    auto const sigma = t.getSigma();

    
    for (int i = 0; i < size; ++i) {
        
        if (sigma(i) <= 0) break;
        z(i) = uTb(i) / sigma(i);
    }

    auto dtheta = t.getV() * z;

    // TODO Perform gradient descent method with line search
    //      to move the end effector toward the target position.
    //
    //   Hint: use the Armature::unpack() and Armature::pack() functions
    //   to set and get the joint angles of the armature.
    // 
    //   Hint: whenever you change the joint angles, you must also call
    //   armature->updateKinematics() to compute the global position.
    //  

    float alpha = 1.f;
    int k = 0;
    Vector<float, -1> theta(dim);
    Vector<float, -1> newTheta(dim);

    //I need a high k because otherwise armijo never solves under very specific position
    //For example, x = -1.077, y = -1.231, z = -1.231 will not solve properly with k=10
    //I suspect some float precision error causes this
    while (k < 20) {            
                
        m_armature->pack(theta);
        newTheta = theta + alpha * dtheta;

        m_armature->unpack(newTheta);
        m_armature->updateKinematics();

        //setting dx inside of getError is pointless here
        if (getError(dx) < error) {
            
            //I removed armijo, I simply break instead
            break;
        }
        else {
            
            m_armature->unpack(theta);
            m_armature->updateKinematics();
            alpha /= 3.f;
        }
        k++;
    }


}
