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

//#include "kalman_local.h"

/// Boost Libraries.
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

using boost::numeric::ublas::identity_matrix;
using boost::numeric::ublas::matrix;

/******************************
**
** #defines
**
******************************/

#ifndef TRUE
#define TRUE 1
#endif /* TRUE */

#ifndef FALSE
#define FALSE 0
#endif /* FALSE */

#ifndef STRING_SIZE
#define STRING_SIZE 64
#endif /* STRING_SIZE */

#ifndef MAX_STATES
#define MAX_STATES 2
#endif /* MAX_STATES */

#ifndef MAX_SENSORS
#define MAX_SENSORS 3
#endif /* MAX_SENSORS */

#ifndef CLEAR_SCREEN
#define CLEAR_SCREEN "\033[2J"
#endif /* CLEAR_SCREEN */

/******************************
**
** Data types
**
******************************/



/******************************
**
** Function prototypes
**
******************************/

//! This function is called when SIGINT (ctrl-c) is invoked.
//! \param signal The SIGINT signal.
void qr_sigint(int signal);

//! Exit function for main program. Sets actuators to safe values and closes
//! all file descriptors. This function is called when SIGINT (ctrl-c) is
//! invoked.
void qr_exit();

//! Print out the values of the matrix.
void print_matrix(matrix<double> *A, std::string _NameString);

//! Main function for the QR program.
//! \param argc Number of command line arguments.
//! \param argv Array of command line arguments.
//! \return Always returns 0.
int main(int argc, char *argv[]);

#endif /* _QR_H_ */
