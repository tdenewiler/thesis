/******************************************************************************
 *
 *  Title:        qr.c
 *
 *  Description:  Main program for training the Q and R covariance matrices.
 *
 *****************************************************************************/

#include "qr.h"

/*------------------------------------------------------------------------------
 * void qr_sigint()
 * Callback for when SIGINT (ctrl-c) is invoked.
 *----------------------------------------------------------------------------*/

void qr_sigint(int signal)
{
    exit(0);
} /* end qr_sigint() */


/*------------------------------------------------------------------------------
 * void qr_exit()
 * Exit function for main program. Sets actuators to safe values and closes all
 * open file descriptors. Callback for when SIGINT (ctrl-c) is invoked.
 *----------------------------------------------------------------------------*/

void qr_exit()
{
    printf("\nQR_EXIT: Shutting down QR program ... ");
    /// Sleep to let things shut down properly.
    usleep(200000);

    /// Close the open file descriptors.

    printf("<OK>\n\n");
} /* end qr_exit() */


/*------------------------------------------------------------------------------
 * int main()
 * Initialize data. Open ports. Run main program loop.
 *----------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    /// Setup exit function. It is called when SIGINT (ctrl-c) is invoked.
    void(*exit_ptr)(void);
    exit_ptr = qr_exit;
    atexit(exit_ptr);

	/// Attach signals to exit function.
    struct sigaction sigint_action;
    sigint_action.sa_handler = qr_sigint;
    sigint_action.sa_flags = 0;
    sigaction(SIGINT, &sigint_action, NULL);
    sigaction(SIGTERM, &sigint_action, NULL);
    sigaction(SIGQUIT, &sigint_action, NULL);
    sigaction(SIGHUP, &sigint_action, NULL);

	/// Declare variables.
	int a = 0;
	int b = 0;
	unsigned int c = 0;
	unsigned int d = 0;
	//unsigned int e = 0;
	//unsigned int f = 0;
	double z = 0.0;
	double minval = 1.;
	double incval = 1.;
	double maxval = 3.;
	int sleeptime = 0;
	bool b_debug = TRUE;
	matrix<double> *Q;
	matrix<double> *R;
	//KalmanLocal *m_pKL = new KalmanLocal("packbot");

    printf("MAIN: Starting QR ... \n");

    /// Initialize variables.
	Q = new matrix<double>(MAX_STATES, MAX_STATES);
	Q->clear();
	R = new matrix<double>(MAX_SENSORS, MAX_SENSORS);
	R->clear();

	/// Start the Kalman filter. Need a small sleep to let the filter start up.
	//m_pKL->Setup();
	//usleep(10000);

	/// Initialize the KF state variables.
	//m_pKL->setState(ACS_X_STATE,0.0);
	//m_pKL->setState(ACS_Y_STATE,0.0);
	//m_pKL->setState(ACS_Z_STATE,0.0);
	//m_pKL->setState(ACS_VELOCITY_STATE,0.0);
	//m_pKL->setState(ACS_PITCH_STATE,0.0);
	//m_pKL->setState(ACS_ROLL_STATE,0.0);
	//m_pKL->setState(ACS_YAW_STATE,0.0);
	//m_pKL->setState(ACS_YAW_SPEED_STATE,0.0);

	for (a = Q->size1()-1; a > -1; a--) {
		for (b = Q->size2()-1; b > -1; b--) {
			for (c = a; c < Q->size1(); c++) {
				for (d = b; d < Q->size2(); d++) {
					for (z = minval; z < maxval + incval; z += incval) {
						(*(Q))(c,d) = z;
						if (b_debug) {
							print_matrix(Q, " Q ");
							usleep(sleeptime);
						}
					}
					(*(Q))(c,d) = 0;
				}
			}
		}
	}

	/*
	for (a = Q->size1()-1; a > -1; --a) {
		printf("MAIN: a = %d\n", a);
		for (b = Q->size2()-1; b > -1; b--) {
			for (c = minval; c < maxval + incval; c += incval) {
				(*(Q))(a,b) = c;
				if (b_debug) {
					print_matrix(Q, " Q ");
					usleep(sleeptime);
				}
			}
			(*(Q))(a,b) = 0;
		}
	}
	//*/


	/*
	for (a = 0; a < Q->size1(); a++) {
		for (b = 0; b < Q->size2(); b++) {
			for (c = minval; c < maxval; c += incval) {
				(*(Q))(a,b) = c;
				if (b_debug) {
					print_matrix(Q, " Q ");
					usleep(sleeptime);
				}
				for (d = a; d < Q->size1(); d++) {
					for (e = b; e < Q->size2(); e++) {
						for (f = minval; f < maxval; f += incval) {
							(*(Q))(d,e) = f;
							if (b_debug) {
								print_matrix(Q, " Q ");
								usleep(sleeptime);
							}
								for (g = minval; g < maxval; g += incval) {
									(*(Q))(Q->size1()-1, Q->size2()-1) = g;
									if (b_debug) {
										print_matrix(Q, " Q ");
										usleep(sleeptime);
									}
								}
							(*(Q))(Q->size1()-1, Q->size2()-1) = 0;
						}
					}
				}
			}
			(*(Q))(a,b) = 0;
		}
	}
	//*/

	return 0;

} /* end main() */


/*------------------------------------------------------------------------------
 * void print_matrix()
 * Print out the values of the matrix.
 *----------------------------------------------------------------------------*/

void print_matrix(matrix<double> *A, std::string _NameString)
{
	unsigned int ii = 0;
	unsigned int jj = 0;

	fprintf(stderr, "\n\rPRINT_MATRIX: %s with size <%d, %d> is:\n\r( ", _NameString.c_str(), (int)A->size1(), (int)A->size2());
	for (ii = 0; ii < A->size1(); ii++) {
		for (jj = 0; jj < A->size2(); jj++) {
			fprintf(stderr, "%f ", (*(A))(ii,jj));
			if ((jj == (A->size2() - 1)) && (ii != (A->size1() - 1))) {
				fprintf(stderr, ")\n\r( ");
			}
		}
	}
	fprintf(stderr, ")\n\r");

	return;
} /* end print_matrix() */
