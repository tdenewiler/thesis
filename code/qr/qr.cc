/***************************************************************************
 *
 *  Title:        qr.cc
 *
 *  Description:  Main program for training the Q and R covariance matrices.
 *
 **************************************************************************/

#include "qr.h"

/*--------------------------------------------------------------------------
 * int main()
 * Initialize data. Open ports. Run main program loop.
 *------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    /// Setup exit function. It is called when SIGINT (ctrl-c) is invoked.
    void(*exit_ptr)(void);
    exit_ptr = qr_exit;
    atexit(exit_ptr);

	/// Attach signals to exit function.
    struct sigaction sigint_action;
    sigint_action.sa_handler = qr_sigint;
    sigemptyset(&sigint_action.sa_mask);
    sigaddset(&sigint_action.sa_mask, SIGINT);
    sigaddset(&sigint_action.sa_mask, SIGHUP);
    sigaddset(&sigint_action.sa_mask, SIGINT);
    sigaddset(&sigint_action.sa_mask, SIGQUIT);
    sigaddset(&sigint_action.sa_mask, SIGILL);
    sigaddset(&sigint_action.sa_mask, SIGABRT);
    sigaddset(&sigint_action.sa_mask, SIGTRAP);
    sigaddset(&sigint_action.sa_mask, SIGIOT);
    sigaddset(&sigint_action.sa_mask, SIGFPE);
    sigaddset(&sigint_action.sa_mask, SIGBUS);
    sigaddset(&sigint_action.sa_mask, SIGSEGV);
    sigaddset(&sigint_action.sa_mask, SIGSYS);
    sigaddset(&sigint_action.sa_mask, SIGPIPE);
    sigaddset(&sigint_action.sa_mask, SIGALRM);
    sigaddset(&sigint_action.sa_mask, SIGTERM);
    sigaddset(&sigint_action.sa_mask, SIGUSR1);
    sigaddset(&sigint_action.sa_mask, SIGUSR2);
    sigaddset(&sigint_action.sa_mask, SIGCHLD);
    sigaddset(&sigint_action.sa_mask, SIGCLD);
    sigaddset(&sigint_action.sa_mask, SIGPWR);
    sigaddset(&sigint_action.sa_mask, SIGXCPU);
    sigint_action.sa_flags = 0;
    sigint_action.sa_restorer = NULL;
    sigaction(SIGINT, &sigint_action, NULL);

	/// Declare variables.
	double minval = 1.;
	double incval = 0.5;
	double maxval = 1.5;
	matrix<double> *Q;
	matrix<double> *R;
	matrix<double> *Qopt;
	matrix<double> *Ropt;

    printf("MAIN: Starting QR ... \n");

    /// Initialize variables.
	Q = new matrix<double>(MAX_STATES, MAX_STATES);
	Q->clear();
	R = new matrix<double>(MAX_SENSORS, MAX_SENSORS);
	R->clear();
	Qopt = new matrix<double>(MAX_STATES, MAX_STATES);
	Qopt->clear();
	Ropt = new matrix<double>(MAX_SENSORS, MAX_SENSORS);
	Ropt->clear();

	/// Cycle through matrices, starting with Q. This will lead to simulated
	/// data as well.
	qr_q(Q, R, Qopt, Ropt, minval, maxval, incval);

	return 0;
} /* end main() */


/*--------------------------------------------------------------------------
 * void print_matrix()
 * Print out the values of the matrix.
 *------------------------------------------------------------------------*/

void print_matrix(matrix<double> *A, std::string _NameString)
{
	unsigned int ii = 0;
	unsigned int jj = 0;

	fprintf(stderr, "\n\rPRINT_MATRIX: %s with size <%d, %d> is:\n\r( ",
		_NameString.c_str(), (int)A->size1(), (int)A->size2());
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


/*--------------------------------------------------------------------------
 * void qr_init_matrix()
 * Initializes A matrix to minval back from the last element to an index
 * element.
 *------------------------------------------------------------------------*/

void qr_init_matrix(matrix<double> *A, double minval,
		int idx_row, int idx_col)
{
	/// Declare variables.
	int i = 0, j = 0;

	/// Set all elements of A from the index row and below to the minimum
	/// value.
	for (i = A->size1()-1; i > idx_row; i--) {
		for (j = A->size2()-1; j > -1; j--) {
			(*(A))(i,j) = minval;
		}
	}

	/// Set all elements of A from the last to the index column to the
	/// minimum value.
	for (j = A->size2()-1; j > idx_col-1; j--) {
		(*(A))(idx_row,j) = minval;
	}

	return;
} /* end qr_init_matrix() */


/*--------------------------------------------------------------------------
 * int qr_find_nonmax()
 * Goes from last to first element looking for element that is not maxval.
 *------------------------------------------------------------------------*/

int qr_find_nonmax(matrix<double> *A, double maxval,
		int *nonmax_row, int *nonmax_col)
{
	/// Declare variables.
	int i = 0, j = 0;

	/// Set all elements of Q to the minimum value.
	for (i = A->size1()-1; i > -1; i--) {
		for (j = A->size2()-1; j > -1; j--) {
			if ((*(A))(i,j) < maxval) {
				*nonmax_row = i;
				*nonmax_col = j;

				return TRUE;
			}
		}
	}

	return FALSE;
} /* end qr_find_nonmax() */


/*--------------------------------------------------------------------------
 * int qr_q()
 * Generates all possible Q matrices.
 *------------------------------------------------------------------------*/

int qr_q(matrix<double> *Q, matrix<double> *R,
		matrix<double> *Qopt, matrix<double> *Ropt,
		double minval, double maxval, double incval)
{
	/// Declare variables.
	int nonmax_remaining = TRUE, done = FALSE, count = 0, count_r = 0;
	int nonmax_row = Q->size1()-1, nonmax_col = Q->size2()-1;
	double tmp = 0.;

	/// Set all elements of Q to the minimum value.
	qr_init_matrix(Q, minval, 0, 0);
	//print_matrix(Q, " Q ");
	count++;
	/// For this Q try all possible R matrices.
	count_r += qr_r(Q, R, Qopt, Ropt, minval, maxval, incval);

	while (!done) {
		/// Look for last element that is less than maxval.
		nonmax_remaining = qr_find_nonmax(Q, maxval,
			&nonmax_row, &nonmax_col);
		(*(Q))(nonmax_row,nonmax_col) += incval;
		/// Check progress.
		//print_matrix(Q, " Q ");
		count++;
		/// For this Q try all possible R matrices.
		count_r += qr_r(Q, R, Qopt, Ropt, minval, maxval, incval);
		if ((*(Q))(nonmax_row,nonmax_col) >= maxval) {
			/// Look for last element that is less than maxval.
			nonmax_remaining = qr_find_nonmax(Q, maxval,
				&nonmax_row, &nonmax_col);
			if (!nonmax_remaining) {
				done = TRUE;
				break;
			}
			/// Save the value of the last nonmax element.
			tmp = (*(Q))(nonmax_row,nonmax_col);
			/// Set all elements to minval except for the index element.
			qr_init_matrix(Q, minval, nonmax_row, nonmax_col);
			(*(Q))(nonmax_row,nonmax_col) = tmp + incval;
			/// Check progress.
			//print_matrix(Q, " Q ");
			count++;
			/// For this Q try all possible R matrices.
			count_r += qr_r(Q, R, Qopt, Ropt, minval, maxval, incval);
		}
	}

	printf("QR_Q: %d Q matrices with %d states generated.\n",
		count, MAX_STATES);
	printf("QR_Q: %d R matrices with %d sensors generated.\n",
		count_r, MAX_SENSORS);

	return count;
} /* end qr_q() */


/*--------------------------------------------------------------------------
 * int qr_r()
 * Generates all possible R matrices with elements in minval:incval:maxval.
 *------------------------------------------------------------------------*/

int qr_r(matrix<double> *Q, matrix<double> *R, matrix<double> *Qopt,
		matrix<double> *Ropt, double minval, double maxval, double incval)
{
	/// Declare variables.
	int nonmax_remaining = TRUE, count = 0, done = FALSE;
	int nonmax_row = R->size1()-1, nonmax_col = R->size2()-1;
	double tmp = 0.;

	/// Set all elements of R to the minimum value.
	qr_init_matrix(R, minval, 0, 0);
	//print_matrix(R, " R ");
	count++;

	while (!done) {
		/// Look for last element that is less than maxval.
		nonmax_remaining = qr_find_nonmax(R, maxval,
			&nonmax_row, &nonmax_col);
		(*(R))(nonmax_row,nonmax_col) += incval;
		/// Check progress.
		//print_matrix(R, " R ");
		count++;
		if ((*(R))(nonmax_row,nonmax_col) >= maxval) {
			/// Look for last element that is less than maxval.
			nonmax_remaining = qr_find_nonmax(R, maxval,
				&nonmax_row, &nonmax_col);
			if (!nonmax_remaining) {
				done = TRUE;
				break;
			}
			/// Save the value of the last nonmax element.
			tmp = (*(R))(nonmax_row,nonmax_col);
			/// Set all elements to minval except for the index element.
			qr_init_matrix(R, minval, nonmax_row, nonmax_col);
			(*(R))(nonmax_row,nonmax_col) = tmp + incval;
			/// Check progress.
			//print_matrix(R, " R ");
			count++;
		}
	}

	return count;
} /* end qr_r() */


/*--------------------------------------------------------------------------
 * void qr_sigint()
 * Callback for when SIGINT (ctrl-c) is invoked.
 *------------------------------------------------------------------------*/

void qr_sigint(int signal)
{
    printf("\nQR_SIGINT: Caught signal %d ", signal);
    char *s_sig = (char *)"QR_SIGINT";

    psignal(signal, s_sig);

    exit(0);
} /* end qr_sigint() */


/*--------------------------------------------------------------------------
 * void qr_exit()
 * Exit function for main program. Sets actuators to safe values and closes
 * all open file descriptors. Callback for when SIGINT (ctrl-c) is invoked.
 *------------------------------------------------------------------------*/

void qr_exit()
{
    printf("\nQR_EXIT: Shutting down QR program ... ");
    /// Sleep to let things shut down properly.
    usleep(200000);

    /// Close the open file descriptors.

    printf("<OK>\n\n");
} /* end qr_exit() */
