/**
 * @file main.cpp
 *
 * @brief Execute unit tests for a simple linear algebra library.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include <gtest/gtest.h>

int main(int argc, char** argv)
{
	// Executer tous les tests unitaires.
	// 
	// Les tests sont écrites dans les fichiers:
	//   tests/Tests1a.cpp
	//   tests/Tests1b.cpp
	//   tests/TestsSupplementaire1a.cpp
	//   tests/TestsSupplementaire1b.cpp
	//
	::testing::InitGoogleTest(&argc, argv);
	const int ret = RUN_ALL_TESTS();

	return ret;
}
