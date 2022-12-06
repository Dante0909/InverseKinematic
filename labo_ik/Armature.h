#pragma once

#include "Math3D.h"
#include <vector>

namespace gti320
{
    class Link
    {
    public:
        // Constructor.
        //  @a _parent = the parent link
        //  @a _eulerAng = the initial rotation
        //  @a _trans = the position relative to the parent
        //
        Link(Link* _parent, const Vector3f& _eulerAng, const Vector3f& _trans);

        // Return true if this link is the root, false otherwise.
        //
        inline bool isRoot() const { return (parent == nullptr);  }

        // Return true if this link is an endEffector, false otherwise.
        //
        inline bool isEndEffector() const { return (enfants.size() == 0); }

        //Get column of the global rigid transformation with a given index
        //
        Vector3f Link::getColumnOfMatrix(const int index) const;            

        // Compute the forward kinematics.
        //
        void forward();

        Vector3f eulerAng;                              // Euler angles giving rotation relative to the parent.
        Vector3f trans;                                 // Translation giving position relative to the parent.
        Matrix4f M;                                     // Global rigid transformation of the link, computed in forward().
        Link* parent;                                   // Parent of this link (if the root, nullptr)
        std::vector<Link*> enfants;                     // Child links.
    };

    class Armature
    {
    public:
        // Constructor.
        //
        Armature();

        // Destructor.
        //
        ~Armature();

        // Forward kinematics to update the global transforms of all links.
        //
        void updateKinematics();

        // Extract the Euler angles of all links and put them in the dense vector @a theta. 
        //
        void pack(Vector<float, Dynamic>& theta);

        // Update the Euler angles of all links on the armature from the vector @a theta.
        // 
        void unpack(const Vector<float, Dynamic>& theta);

        inline Link* getEndAffector() const { return links.at(links.size() - 1); }

        std::vector<Link*> links;                       // All of the articulated links that make-up the armature.
        Link* root;                                     // The root link.
    };
}

