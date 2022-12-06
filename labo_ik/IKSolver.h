    #pragma once

#include "Math3D.h"

namespace gti320
{
    class Armature;

    typedef Matrix<float, 3, Dynamic, ColumnStorage> Jacobianf;


    // An inverse kinematics solver for a simple armature.
    //
    class IKSolver
    {
    public:
        // Constructor.
        // @a _armature = The armature (see class Armature).
        // @a _targetPos = A reference to the target position of the end effector.
        //
        IKSolver(Armature* _armature, Vector3f& _targetPos);

        // Execute one solver step.
        // A gradient descent method with line search is used.
        // Euler angles of the armature links are automatically computed and updated.
        // 
        void solve();

        // Return the error as the distance between the end effector and the target position.
        // The displacement vector @a _dx is updated with the value:
        //   _dx = m_targetPos - f(theta) 
        //
        float getError(Vector3f& _dx) const;

    private:
        Armature* m_armature;                       // The armature.
        Vector3f& m_targetPos;                      // The target position of the end effector.
        Jacobianf m_J;                              // The end effector Jacobian.
	};
	
}

