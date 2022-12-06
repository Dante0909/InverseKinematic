#pragma once

/**
 * @file Math3D.h
 *
 * @brief Fonctions pour l'initialisation et la manipulation de matrices de
 * rotation, de matrices de transformations en coordonnées homogènes et de
 * vecteurs 3D.
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */
#include <iostream>
#include "Matrix.h"
#include "Operators.h"
#include "Vector.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>

namespace gti320 {

    // Deux types de vecteurs 3D considérés ici
    typedef Vector<double, 3> Vector3d;
    typedef Vector<float, 3> Vector3f;

    // Dans le cadre de ce projet, nous considérons seulement deux
    // cas :
    //
    //  - les rotations
    //  - les translations
    //
    // Deux types de matrices en coordonnées homogèes :
    typedef Matrix<double, 4, 4, ColumnStorage> Matrix4d;
    typedef Matrix<float, 4, 4, ColumnStorage> Matrix4f;
    //
    // Deux types de matrices pour les rotations
    typedef Matrix<double, 3, 3, ColumnStorage> Matrix3d;
    typedef Matrix<float, 3, 3, ColumnStorage> Matrix3f;

    /** Initialise et retourne la matrice identité */
    template<>
    inline void Matrix4d::setIdentity()
    {
        //I can assume the size is 4 and is of type double
        memset(m_storage.data(), 0, m_storage.size() * sizeof(double));

        for (int i = 0; i < 4; ++i) {
            (*this)(i, i) = 1;
        }
        /** TODO: Redéfinir la fonction membre de façon optimisée pour les Matrix4d */
    }
    template<>
    inline void Matrix4f::setIdentity() {
        //new function to set identity to Matrix4f
        memset(m_storage.data(), 0, m_storage.size() * sizeof(float));

        for (int i = 0; i < 4; ++i) {
            (*this)(i, i) = 1;
        }
    }
    

    /**
     * Calcul de la matrice inverse SPÉCIALISÉ pour le cas
     * d'une matrice de transformation en coordonnées homogènes.
     */
    template<>
    inline Matrix4d Matrix4d::inverse() const
    {
        Matrix4d buffer = (*this);
        auto Rt = buffer.block(0, 0, 3, 3).transpose<double, 3, 3, ColumnStorage>();
        buffer.block(0, 0, 3, 3) = Rt;

        Matrix<double, 3, 1, ColumnStorage> A;
        A = buffer.block(0, 3, 3, 1);

        buffer.block(0, 3, 3, 1) = ((Scalar)-1.) * Rt * A;

       
        return buffer;
        /** TODO: Implémenter. */
    }

    /**
     * Calcul de la matrice inverse SPÉCIALISÉ pour le cas
     * d'une matrice de rotation.
     */
    template<>
    inline Matrix3d Matrix3d::inverse() const
    { 
        //the inverse of a rotation matrix is the same its transpose
        return this->transpose<double,3,3,ColumnStorage>();
        /** TODO: Implémenter. */
    }
    template<>
    inline Matrix3f Matrix3f::inverse() const
    {
        //new function to inverse Matrix3f
        return this->transpose<float, 3, 3, ColumnStorage>();
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication d'une matrice 4x4 avec un vecteur 3D où la matrice
     * représente une transformation en coordonnées homogène.
     */
    template<typename _Scalar>
    Vector<_Scalar, 3> operator*(const Matrix<_Scalar, 4, 4, ColumnStorage>& A, const Vector<_Scalar, 3>& v)
    {
        Matrix3d rotation;
        rotation = A.block(0, 0, 3, 3);

        Vector<_Scalar, 3> translation;

        translation(0) = A(0, 3);
        translation(1) = A(1, 3);
        translation(2) = A(2, 3);

        return rotation * v + translation;

        /** TODO: Implémenter. */
    }

    /**
     * Créer et retourne une matrice de rotation définie par les angles
     * d'Euler XYZ exprimés en radians.
     *
     * La matrice doit correspondre au produit : Rz * Ry * Rx.
     */
    template<typename _Scalar>
    static Matrix<_Scalar, 3, 3> makeRotation(_Scalar x, _Scalar y, _Scalar z)
    {
        Matrix<_Scalar, 3, 3> R;
        _Scalar cosX = std::cos(x);
        _Scalar sinX = std::sin(x);
        _Scalar cosY = std::cos(y);
        _Scalar sinY = std::sin(y);
        _Scalar cosZ = std::cos(z);
        _Scalar sinZ = std::sin(z);        

        R(0, 0) = cosY * cosZ;
        R(0, 1) = cosZ * sinX * sinY - cosX * sinZ;
        R(0, 2) = cosX * cosZ * sinY + sinX * sinZ;
        R(1, 0) = cosY * sinZ;
        R(1, 1) = cosX * cosZ + sinX * sinY * sinZ;
        R(1, 2) = cosX * sinY * sinZ - cosZ * sinX;
        R(2, 0) = -sinY;
        R(2, 1) = cosY * sinX;
        R(2, 2) = cosX * cosY;
        return R;
        /** TODO: Implémenter. */
    }

    //NOUVELLE
    //A function that calculates the cross product of two vectors
    //Used to build the jacobian matrix
    static Vector3f crossProduct(Vector3f a, Vector3f b) {
        Vector3f cross;
        cross(0) = (a(1) * b(2)) - (a(2) * b(1));
        cross(1) = (a(2) * b(0)) - (a(0) * b(2));
        cross(2) = (a(0) * b(1)) - (a(1) * b(0));
        return cross;
    }

} // namespace gti320
