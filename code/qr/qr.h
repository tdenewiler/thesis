/**
 *  \file qr.h
 *  \brief Main program for training the Q and R covariance matrices.
 */

#ifndef _QR_H_
#define _QR_H_

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>

/// Boost Libraries.
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using boost::numeric::ublas::identity_matrix;
using boost::numeric::ublas::matrix;

/******************************
 *
 * #defines
 *
 *****************************/

#ifndef TRUEFALSE
#define TRUE 1
#define FALSE 0
#endif /* TRUEFALSE */

#ifndef MAX_STATES
#define MAX_STATES 4
#endif /* MAX_STATES */

#ifndef MAX_SENSORS
#define MAX_SENSORS 3
#endif /* MAX_SENSORS */

#ifndef CLEAR_SCREEN
#define CLEAR_SCREEN "\033[2J"
#endif /* CLEAR_SCREEN */

/******************************
 *
 * Data types
 *
 *****************************/



/******************************
 *
 * Function prototypes
 *
 *****************************/

//! This function is called when SIGINT (ctrl-c) is invoked.
//! \param signal The SIGINT signal.
void qr_sigint(int signal);

//! Exit function for main program. Sets actuators to safe values and closes
//! all file descriptors. This function is called when SIGINT (ctrl-c) is
//! invoked.
void qr_exit();

//! Print out the values of the matrix.
//! \param A The matrix to print out.
//! \param _NameString The name of the matrix.
void print_matrix(matrix<double> *A, std::string _NameString);

//! Generates all possible Q matrices with elements in minval:incval:maxval.
//! \param Q The Q covariance matrix.
//! \param R The R covariance matrix.
//! \param Qopt The optimal Q matrix. It is replaced when a better error
//! metric is found.
//! \param Ropt The optimal R matrix. It is replaced when a better error
//! metric is found.
//! \param minval The minimum value to perform the search over.
//! \param maxval The maximum value to perform the search over.
//! \param incval The increment value or step size in going from minval to
//! maxval.
//! \return The number of matrices generated.
int qr_q(matrix<double> *Q, matrix<double> *R, matrix<double> *Qopt,
		matrix<double> *Ropt, double minval, double maxval, double incval);

//! Generates all possible R matrices with elements in minval:incval:maxval.
//! \param Q The Q covariance matrix.
//! \param R The R covariance matrix.
//! \param Qopt The optimal Q matrix. It is replaced when a better error
//! metric is found.
//! \param Ropt The optimal R matrix. It is replaced when a better error
//! metric is found.
//! \param minval The minimum value to perform the search over.
//! \param maxval The maximum value to perform the search over.
//! \param incval The increment value or step size in going from minval to
//! maxval.
//! \return The number of matrices generated.
int qr_r(matrix<double> *Q, matrix<double> *R, matrix<double> *Qopt,
		matrix<double> *Ropt, double minval, double maxval, double incval);

//! Goes from last to first element looking for element that is not maxval.
//! \param A The matrix to search through.
//! \param maxval The value to search for.
//! \param nonmax_row A pointer that will take in the row of the nonmax
//! element.
//! \param nonmax_col A pointer that will take in the column of the nonmax
//! element.
//! \return TRUE if element found that is not maxval, else FALSE.
int qr_find_nonmax(matrix<double> *A, double maxval,
		int *nonmax_row, int *nonmax_col);

//! Initializes A matrix to minval back from the last element to an index
//! element.
//! \param A The matrix to initialize.
//! \param minval The value to initialize the elements of A to.
//! \param idx_row Only elements after this row will be initialized.
//! \param idx_col Only elements in the column after this one and in all
//! lower rows will be initialized.
void qr_init_matrix(matrix<double> *A, double minval,
		int idx_row, int idx_col);

//! Main function for the QR program.
//! \param argc Number of command line arguments.
//! \param argv Array of command line arguments.
//! \return Always returns 0.
int main(int argc, char *argv[]);

#endif /* _QR_H_ */
