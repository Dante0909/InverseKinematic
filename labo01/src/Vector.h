#pragma once

/**
 * @file Vector.h
 *
 * @brief Implémentation de vecteurs simples
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */

#include "MatrixBase.h"

namespace gti320 {

    /**
     * Classe vecteur générique.
     *
     * Cette classe réutilise la classe `MatrixBase` et ses spécialisations de
     * templates pour les manipulation bas niveau.
     */
    template<typename _Scalar = double, int _Rows = Dynamic>
    class Vector : public MatrixBase<_Scalar, _Rows, 1> {
    public:
        /** Constructeur par défaut */
        Vector() : MatrixBase<_Scalar, _Rows, 1>() { }

        /** Contructeur à partir d'un taille (rows). */
        explicit Vector(int rows) : MatrixBase<_Scalar, _Rows, 1>(rows, 1) { }

        /** Constructeur de copie */
        Vector(const Vector& other) : MatrixBase<_Scalar, _Rows, 1>(other) {}

        /** Destructeur */
        ~Vector() {}

        /** Opérateur de copie */
        Vector& operator=(const Vector& other)
        {
            MatrixBase<_Scalar, _Rows, 1>::operator=(other);
            /*if(this != &other){
                this->m_storage = other.m_storage;
            */
            
            return *this;
        }

        /** Accesseur à une entrée du vecteur (lecture seule) */
        _Scalar operator()(int i) const
        {
            assert(i >= 0 && i <= this->size());
                return m_storage.data()[i];
            
        }

        /** Accesseur à une entrée du vecteur (lecture et écriture) */
        _Scalar& operator()(int i)
        {
            assert(i >= 0 && i < this->size());
            return m_storage.data()[i];
            
            /** TODO: Implémenter. */
        }

        /** Modifie le nombre de lignes du vecteur */
        void resize(int _rows)
        {
            MatrixBase<_Scalar, _Rows, 1>::resize(_rows, 1);
        }

        /** Produit scalaire de *this et other */
        inline _Scalar dot(const Vector& other) const
        {
            _Scalar dot = 0.0;
            for (int i = 0; i < this->size(); ++i) {                
                dot += (*this)(i) * other(i);
            }
            /** TODO: Implémenter. */
            return dot;
        }

        /** Retourne la norme euclidienne du vecteur */
        inline _Scalar norm() const
        {
            /** TODO: Implémenter. */
            //the dot product of a vector is the norm^2
            return std::sqrt(dot(*this));
        }
    };
} // namespace gti320
