#pragma once

/**
 * @file Operators.h
 *
 * @brief Implémentation de divers opérateurs arithmétiques pour les matrices et les vecteurs.
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */

#include "Matrix.h"
#include "Vector.h"

namespace gti320 {

    /** Multiplication : Matrice * Matrice (générique) */
    template<typename _Scalar, int RowsA, int ColsA, int StorageA, int RowsB, int ColsB, int StorageB>
    Matrix<_Scalar, RowsA, ColsB> operator*(const Matrix<_Scalar, RowsA, ColsA, StorageA>& A, const Matrix<_Scalar, RowsB, ColsB, StorageB>& B)
    {
        //same implementation as row * col
        assert(A.cols() == B.rows());
        int const rows = A.rows();
        int const cols = B.cols();
        int const brows = B.rows();
        Matrix<_Scalar, RowsA, ColsB> C;
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {
                for (int k = 0; k < brows; ++k) {
                    C(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        
        return C;
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication : Matrice (colonne) * Matrice (ligne)
     *
     * Spécialisation de l'opérateur de multiplication pour le cas où les matrices
     * ont un stockage à taille dynamique et où la matrice de gauche utilise un
     * stockage par colonnes et celle de droite un stockage par lignes.
     */
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
    {
        assert(A.cols() == B.rows());
        int rows = A.rows();
        int cols = B.cols();
        Matrix<_Scalar, Dynamic, Dynamic> C(rows, cols);


        for (int k = 0; k < rows; ++k) {
            for (int j = 0; j < cols; ++j) {
                for (int i = 0; i < rows; ++i) {

                    C(i, j) += A(i, k) * B(k, j);
                }
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication : Matrice (ligne) * Matrice (colonne)
     *
     * Spécialisation de l'opérateur de multiplication pour le cas où les matrices
     * ont un stockage à taille dynamique et où la matrice de gauche utilise un
     * stockage par lignes et celle de droite un stockage par colonnes.
     */
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator*(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
    {
        assert(A.cols() == B.rows());
        int rows = A.rows();
        int cols = B.cols();
        Matrix<_Scalar, Dynamic, Dynamic> C(rows, cols);
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {
                for (int k = 0; k < rows; ++k) {
                    C(i, j) += A(i,k) * B(k,j);
                }                  
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    /** Addition : Matrice + Matrice (générique) */
    template<typename _Scalar, int Rows, int Cols, int StorageA, int StorageB>
    Matrix<_Scalar, Rows, Cols> operator+(const Matrix<_Scalar, Rows, Cols, StorageA>& A, const Matrix<_Scalar, Rows, Cols, StorageB>& B)
    {
        //same implementation as col + col
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Rows,Cols> C(rows, cols);
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < cols; ++i) {
            
                C(i, j) = A(i, j) + B(i, j);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    /**
     * Addition : Matrice (colonne) + Matrice (colonne)
     *
     * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
     * sont stockées par colonnes.
     */
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator+(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Dynamic, Dynamic> C(rows, cols);

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                C(j, i) = A(j, i) + B(j,i);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }
  
    /**
     * Addition : Matrice (ligne) + Matrice (ligne)
     *
     * Spécialisation de l'opérateur d'addition pour le cas où les deux matrices
     * sont stockées par lignes.
     */
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic, RowStorage> operator+(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Dynamic, Dynamic, RowStorage> C(cols, rows);
        for (int i = 0; i < rows;  ++i) {
            for (int j = 0; j < cols; ++j) {
                C(i, j) = A(i, j) + B(i, j);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication  : Scalaire * Matrice (colonne)
     *
     * Spécialisation de l'opérateur de multiplication par un scalaire pour le
     * cas d'une matrice stockée par colonnes.
     */
    template<typename _Scalar, int _Rows, int _Cols>
    Matrix<_Scalar, _Rows, _Cols, ColumnStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A)
    {
        Matrix<_Scalar, _Rows, _Cols, ColumnStorage> B(A.rows(),A.cols());
        for (int j = 0; j < A.rows(); ++j) {
            for (int i = 0; i < A.cols(); ++i) {
                            
                B(i,j) = A(i,j) * a;
            }
        }
        return B;
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication  : Scalaire * Matrice (ligne)
     *
     * Spécialisation de l'opérateur de multiplication par un scalaire pour le
     * cas d'une matrice stockée par lignes.
     */
    template<typename _Scalar, int _Rows, int _Cols>
    Matrix<_Scalar, _Rows, _Cols, RowStorage> operator*(const _Scalar& a, const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A)
    {
        Matrix<_Scalar, _Rows, _Cols, RowStorage> B(A.rows(), A.cols());
        for (int i = 0; i < A.rows(); ++i) {
            for (int j = 0; j < A.cols(); ++j) {

                B(i, j) = A(i, j) * a;
            }
        }
        return B;
        /** TODO: Implémenter. */
    }

    /**
     * Multiplication : Matrice (ligne) * Vecteur
     *
     * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
     * cas où la matrice est représentée par lignes.
     */
    template<typename _Scalar, int _Rows, int _Cols>
    Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, RowStorage>& A, const Vector<_Scalar, _Cols>& v)
    {
        assert(A.cols() == v.rows());

        Vector<_Scalar, _Rows> W(A.rows());
        for (int i = 0; i < A.rows(); ++i) {
            for (int k = 0; k < A.cols(); ++k) {
                W(i) += A(i, k) * v(k);
            }

        }
        return W;
        /** TODO: Implémenter. */
    }
    
    /**
     * Multiplication : Matrice (colonne) * Vecteur
     *
     * Spécialisation de l'opérateur de multiplication matrice*vecteur pour le
     * cas où la matrice est représentée par colonnes.
     */
    template<typename _Scalar, int _Rows, int _Cols>
    Vector<_Scalar, _Rows> operator*(const Matrix<_Scalar, _Rows, _Cols, ColumnStorage>& A, const Vector<_Scalar, _Cols>& v)
    {
        assert(A.cols() == v.rows());

        Vector<_Scalar, _Rows> W(A.rows());

        for (int k = 0; k < A.cols(); ++k) {
            for (int i = 0; i < A.rows(); ++i) {

                W(i) += A(i, k) * v(k);
            }

        }
        return W;
        /** TODO: Implémenter. */
    }

    /** Multiplication : Scalaire * Vecteur */
    template<typename _Scalar, int _Rows>
    Vector<_Scalar, _Rows> operator*(const _Scalar& a, const Vector<_Scalar, _Rows>& v)
    {
        Vector<_Scalar, _Rows> buffer(v.size());

        for (int i = 0; i < v.size(); ++i) {
            buffer(i) = v(i) * a;
        }
        return buffer;
        /** TODO: Implémenter. */
    }

    //Produit vectoriel de Hadamard
    template<typename _Scalar>
    Vector<_Scalar> operator*(const Vector<_Scalar>& v, const Vector<_Scalar>& w) {
        assert(v.size() == w.size());
        Vector<_Scalar> buffer(v.size());
        for (int i = 0; i < v.size(); ++i) {
            buffer(i) = v(i) * w(i);
        }
        return buffer;
    }

    /** Addition : Vecteur + Vecteur */
    template<typename _Scalar, int _RowsA, int _RowsB>
    Vector<_Scalar, _RowsA> operator+(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
    {
        int size = a.size();
        assert(size == b.size());
        Vector<_Scalar, _RowsA> buffer(size);

        for (int i = 0; i < size; ++i) {
            buffer(i) = a(i) + b(i);
        }
        return buffer;
        /** TODO: Implémenter. */
    }

    /** Soustraction : Vecteur - Vecteur */
    template<typename _Scalar, int _RowsA, int _RowsB>
    Vector<_Scalar, _RowsA> operator-(const Vector<_Scalar, _RowsA>& a, const Vector<_Scalar, _RowsB>& b)
    {
        int size = a.size();
        assert(size == b.size());
        Vector<_Scalar, _RowsA> buffer(size);
        for (int i = 0; i < size; ++i) {
            buffer(i) = a(i) - b(i);
        }
        return buffer;

        /** TODO: Implémenter. */
    }

    //Generic matrix - generic matrix
    template<typename _Scalar, int Rows, int Cols, int StorageA, int StorageB>
    Matrix<_Scalar, Rows, Cols> operator-(const Matrix<_Scalar, Rows, Cols, StorageA>& A, const Matrix<_Scalar, Rows, Cols, StorageB>& B)
    {
        //same implementation as col - col
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Rows, Cols> C(rows, cols);

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                C(j, i) = A(j, i) - B(j, i);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    // Col matrix - col matrix
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic> operator-(const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, ColumnStorage>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Dynamic, Dynamic> C(rows, cols);

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                C(j, i) = A(j, i) - B(j, i);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

    // Row matrix - row matrix
    template<typename _Scalar>
    Matrix<_Scalar, Dynamic, Dynamic, RowStorage> operator-(const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& A, const Matrix<_Scalar, Dynamic, Dynamic, RowStorage>& B)
    {
        assert(A.cols() == B.cols() && A.rows() == B.rows());
        int cols = A.cols();
        int rows = B.rows();
        Matrix<_Scalar, Dynamic, Dynamic, RowStorage> C(cols, rows);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                C(i, j) = A(i, j) - B(i, j);
            }
        }
        return C;
        /** TODO: Implémenter. */
    }

} // namespace gti320
