#pragma once

/**
 * @file Matrix.h
 *
 * @brief Implémentation de matrices simples.
 *
 * Nom: Mathias Glorieux
 * Code permanent : AR53050
 * Email : mathias.glorieux.1@ens.etsmtl.ca
 *
 */

#include "MatrixBase.h"

namespace gti320 {
	enum StorageType {
		ColumnStorage = 0,
		RowStorage = 1
	};

	// Déclaration avancée
	template<typename _Scalar, int _RowsAtCompile, int _ColsAtCompile, int _StorageType>
	class SubMatrix;

	/**
	 * Classe Matrix spécialisé pour le cas générique. (defaut par colonne)
	 *
	 * Note :
	 * Le cas d'un stockage par ligne fait l'objet d'une spécialisation de patron. (voir plus bas)
	 */
	template<typename _Scalar = double, int _RowsAtCompile = Dynamic, int _ColsAtCompile = Dynamic, int _StorageType = ColumnStorage>
	class Matrix : public MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile> {
	public:
		/** Constructeur par défaut */
		Matrix() : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>() { }

		/** Constructeur de copie */
		Matrix(const Matrix& other) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(other) {}

		/** Constructeur avec spécification du nombre de ligne et de colonnes */
		explicit Matrix(int _rows, int _cols) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(_rows, _cols) { }

		/** Destructeur */
		~Matrix() {}

		/**
		 * Opérateur de copie à partir d'une sous-matrice.
		 * If the size of column or rows is not dynamic, the size of the matrix has to be the same
		 * Exemple :
		 * Matrix<...> A(...);
		 * Matrix<...> B(...);
		 * B = A.block(i,j,m,n);
		 */
		template<typename _OtherScalar, int OtherRows, int _OtherCols, int _OtherStorage>
		Matrix& operator=(const SubMatrix<_OtherScalar, OtherRows, _OtherCols, _OtherStorage>& submatrix)
		{
			int rows = submatrix.rows();
			int cols = submatrix.cols();
			if constexpr (_RowsAtCompile == Dynamic && _ColsAtCompile == Dynamic) {
				if (this->rows() != rows || this->cols() != cols) {
					this->resize(rows, cols);
				}
			}
			else { 
				assert(this->rows() == submatrix.rows() && this->cols() == submatrix.cols());
			}
			
			for (int i = 0; i < cols; ++i) {
				for (int j = 0; j < rows; ++j) {
					(*this)(j, i) = submatrix(j, i);					
				}
			}
			/**
			 * TODO: Copier les données de la sous-matrice.
			 *
			 * Note :
			 * Si les dimensions ne correspondent pas, la matrice doit être redimensionnée.
			 * Vous pouvez présumer qu'il s'agit d'un stockage par colonnes.
			 */
			return *this;
		}

		/** Accesseur à une entrée de la matrice (lecture seule) */
		_Scalar operator()(int i, int j) const
		{
			assert(i >= 0 && i < this->rows());
			assert(j >= 0 && j < this->cols());
			return m_storage.data()[j * this->rows() + i];
			/** TODO: Implémenter. */

		}

		/** Accesseur à une entrée de la matrice (lecture ou écriture) */
		_Scalar& operator()(int i, int j)
		{
			assert(i >= 0 && i < this->rows());
			assert(j >= 0 && j < this->cols());
			return m_storage.data()[j * this->rows() + i];
			/**
			 * TODO: Implémenter.
			 *
			 * Indice :
			 * L'implémentation devrait être identique à celle de la fonction précédente.
			 */
		}
		/** Créer une sous-matrice de taille (rows, cols) à partir de l'index (i,j) */
		SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType> block(int i, int j, int rows, int cols) const
		{
			assert(i >= 0 && rows > 0);
			assert(j >= 0 && cols > 0);
			assert(i + rows <= this->rows());
			assert(j + cols <= this->cols());
			return SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>(*this, i, j, rows, cols);
		}

		/** Calcule l'inverse de la matrice */
		Matrix inverse() const
		{
			// Ne rien faire.
			return *this;
		}

		/** Retourne la transposée de la matrice */
		template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
		Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> transpose() const
		{
			int rows = this->rows();
			int cols = this->cols();
			assert(_OtherRows == cols || _OtherRows == Dynamic);
			assert(_OtherCols == rows || _OtherCols == Dynamic);
			Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> buffer(cols, rows);

			for (int i = 0; i < this->rows(); ++i) {
				for (int j = 0; j < this->cols(); ++j) {
					buffer(j, i) = (*this)(i, j);
				}
			}
			return buffer;
			/** TODO: Calculer et retourner la transposée de la matrice. */
		}

		/** Affecte l'identité à la matrice */
		inline void setIdentity()
		{
			int max = this->cols() < this->rows() ? this->cols() : this->rows();
			this->setZero();

			for (int i = 0; i < max; ++i) {
				(*this)(i, i) = 1;
			}
			/**
			 * TODO: Affecter la valeur 0.0 partout, sauf sur la diagonale principale où c'est 1.0.
			 *
			 * Note :
			 * Votre implémentation devrait aussi fonctionner pour des matrices qui ne sont pas carrées.
			 */
		}
	};

	/** Classe Matrix spécialisée pour un stockage par lignes */
	template<typename _Scalar, int _RowsAtCompile, int _ColsAtCompile>
	class Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage> : public MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile> {
	public:
		/** Constructeur par défaut */
		Matrix() : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>() {}

		/** Constructeur de copie */
		Matrix(const Matrix& other) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(other) {}

		/** Constructeur avec spécification du nombre de ligne et de colonnes */
		explicit Matrix(int rows, int cols) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(rows, cols) {}

		/** Destructeur */
		~Matrix() {}

		/**
		 * Opérateur de copie à partir d'une sous-matrice.
		 * Exemple :
		 * Matrix<...> A(...);
		 * Matrix<...> B(...);
		 * B = A.block(i,j,m,n);
		 */
		template<typename _OtherScalar, int OtherRows, int _OtherCols, int _OtherStorage>
		Matrix& operator=(const SubMatrix<_OtherScalar, OtherRows, _OtherCols, _OtherStorage>& submatrix)
		{
			int rows = submatrix.rows();
			int cols = submatrix.cols();
			if constexpr (_RowsAtCompile == Dynamic || _ColsAtCompile == Dynamic) {
				if (this->rows() != rows || this->cols() != cols) {
					this->resize(rows, cols);
				}
			}
			else {
				assert(this->rows() == submatrix.rows() && this->cols() == submatrix.cols());
			}
			for (int i = 0; i < rows; ++i) {
				for (int j = 0; j < cols; ++j) {
					(*this)(i, j) = submatrix(i, j);
				}
			}
			//I was trying to use memcpy but I couldn't find a way how. For instance, submatrix.m_matrix is not accessible
		
			/**
			 * TODO: Copier les données de la sous-matrice.
			 *
			 * Note :
			 * Si les dimensions ne correspondent pas, la matrice doit être redimensionnée.
			 * Vous pouvez présumer qu'il s'agit d'un stockage par lignes.
			 */
			return *this;
		}

		/** Accesseur à une entrée de la matrice (lecture seule) */
		_Scalar operator()(int i, int j) const
		{
			assert(i >= 0 && i < this->rows());
			assert(j >= 0 && j < this->cols());
			return m_storage.data()[i * this->cols() + j];

			/** TODO: Implémenter. */
		}

		/** Accesseur à une entrée de la matrice (lecture ou écriture) */
		_Scalar& operator()(int i, int j)
		{
			assert(i >= 0 && i < this->rows());
			assert(j >= 0 && j < this->cols());
			return m_storage.data()[i * this->cols() + j];

			/** TODO: Implémenter. */
		}

		/** Créer une sous-matrice de taille (rows, cols) à partir de l'index (i,j) */
		SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage> block(int i, int j, int rows, int cols) const
		{
			assert(i >= 0 && rows > 0);
			assert(j >= 0 && cols > 0);
			assert(i + rows <= this->rows());
			assert(j + cols <= this->cols());
			return SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage>(*this, i, j, rows, cols);
		}

		/** Calcule l'inverse de la matrice */
		Matrix inverse() const
		{
			// Ne rien faire.
			return *this;
		}

		/** Retourne la transposée de la matrice */
		Matrix<_Scalar, _ColsAtCompile, _RowsAtCompile, ColumnStorage> transpose() const
		{
			Matrix<_Scalar, _ColsAtCompile, _RowsAtCompile, ColumnStorage> buffer(this->cols(), this->rows());

			for (int i = 0; i < this->rows(); ++i) {
				for (int j = 0; j < this->cols(); ++j) {
					buffer(j, i) = (*this)(i, j);

				}
			}
			return buffer;
			/**
			 * TODO:
			 * - Calculer et retourner la transposée de la matrice.
			 * - Optimisez cette fonction en tenant compte du type de stockage utilisé.
			 */
		}

		/** Affecte l'identité à la matrice */
		inline void setIdentity()
		{
			int max = this->cols() < this->rows() ? this->cols() : this->rows();
			memset(m_storage.data(), 0, m_storage.size() * sizeof(_Scalar));

			for (int i = 0; i < max; ++i) {
				(*this)(i, i) = 1;
			}
			/**
			 * TODO: Affecter la valeur 0.0 partout, sauf sur la diagonale principale où c'est 1.0.
			 *
			 * Note :
			 * Votre implémentation devrait aussi fonctionner pour des matrices qui ne sont pas carrées.
			 */
		}
	};

	/**
	 * Classe pour accéder à une sous-matrice.
	 *
	 * Note :
	 * Un sous-matrice ne copie pas les données.
	 * Elle conserve plutôt une référence à la matrice originale.
	 */
	template<typename _Scalar, int _RowsAtCompile, int _ColsAtCompile, int _StorageType>
	class SubMatrix {
	private:
		// Référence à la matrice originale
		Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& m_matrix;

		// Constructeur par défaut (privé)
		SubMatrix() {}

		// (m_i, m_j) est le coin supérieur gauche de la sous-matrice dans la matrice originale.
		int m_i; // Décalage en ligne
		int m_j; // Décalage en colonne

		// La sous-matrice est de dimension : m_rows x m_cols
		int m_rows; // Hauteur de la sous-matrice (nombre de lignes)
		int m_cols; // Largeur de la sous-matrice (nombre de colonnes)

	public:
		/** Constructeur à partir d'une référence en lecture seule à une matrice. */
		SubMatrix(const Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& _matrix, int _i, int _j, int _rows, int _cols) :
			m_matrix(const_cast<Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>&>(_matrix)),
			m_i(_i),
			m_j(_j),
			m_rows(_rows),
			m_cols(_cols)
		{
		}

		/** Constructeur à partir d'une référence en lecture et écriture à une matrice. */
		explicit SubMatrix(Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& _matrix, int _i, int _j, int _rows, int _cols) :
			m_matrix(_matrix),
			m_i(_i),
			m_j(_j),
			m_rows(_rows),
			m_cols(_cols)
		{
		}

		/** Constructeur de copie */
		SubMatrix(const SubMatrix& other) :
			m_matrix(other.m_matrix),
			m_i(other.m_i),
			m_j(other.m_j),
			m_rows(other.m_rows),
			m_cols(other.m_cols)
		{
		}

		/** Destructeur */
		~SubMatrix() {}

		/** Opérateur de copie (à partir d'une matrice) */
		template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
		SubMatrix& operator=(const Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage>& matrix)
		{
			assert(m_rows == matrix.rows());
			assert(m_cols == matrix.cols());
			for (int i = 0; i < matrix.rows(); ++i) {
				for (int j = 0; j < matrix.cols(); ++j) {

					m_matrix(i + m_i, j + m_j) = matrix(i, j);
				}
			}
			/**
			 * TODO: Copier les valeurs de la matrice dans la sous-matrice.
			 *
			 * Note :
			 * Les dimensions de la matrice doivent correspondre à celle de la sous-matrice.
			 */
			return *this;
		}

		/** Accesseur aux entrées de la sous-matrice (lecture seule) */
		_Scalar operator()(int i, int j) const
		{
			assert(i >= 0 && i < m_rows);
			assert(j >= 0 && j < m_cols);
			return m_matrix(i + m_i, j + m_j);//.data()[i * this->cols() + j];

			/**
			 * TODO: Implémenter.
			 *
			 * Note :
			 * Il faut s'assurer que les indices respectent la taille de la sous-matrice.
			 * */
		}

		/** Accesseur aux entrées de la sous-matrice (lecture et écriture) */
		_Scalar& operator()(int i, int j)
		{
			assert(i >= 0 && i < m_rows);
			assert(j >= 0 && j < m_cols);
			return m_matrix(i + m_i, j + m_j);
			/**
			 * TODO: Implémenter.
			 *
			 * Note :
			 * Il faut s'assurer que les indices respectent la taille de la sous-matrice.
			 * */
		}

		/** Retourne la transposée de la sous-matrice sous la forme d'une matrice. */
		template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
		Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> transpose() const
		{			
			int rows = this->rows();
			int cols = this->cols();
			assert(_OtherRows == cols || _OtherRows == Dynamic);
			assert(_OtherCols == rows || _OtherCols == Dynamic);
			Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> buffer(cols, rows);

			for (int i = 0; i < rows; ++i) {
				for (int j = 0; j < cols; ++j) {
					buffer(j, i) = (*this)(i, j);
				}
			}
			return buffer;
			/** TODO: Implémenter. */
		}

		inline int rows() const { return m_rows; }
		inline int cols() const { return m_cols; }
	};

} // namespace gti320
