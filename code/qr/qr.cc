/*-------------------------------------------------------
 *
 *  Title:        qr.cc
 *
 *  Description:  Main program for training the Q and R
 *				  covariance matrices.
 *
 *-----------------------------------------------------*/

#include "qr.h"

/*-------------------------------------------------------
 * void qr_sigint()
 * Callback for when SIGINT (ctrl-c) is invoked.
 *-----------------------------------------------------*/

void qr_sigint(int signal)
{
	printf("\nQR_SIGINT: Caught signal %d ", signal);
	char *s_sig = (char *)"QR_SIGINT";

	psignal(signal, s_sig);

	exit(0);
} /* end qr_sigint() */


/*-------------------------------------------------------
 * void qr_exit()
 * Exit function for main program. Sets actuators to safe
 * values and closes all open file descriptors. Callback
 * for when SIGINT (ctrl-c) is invoked.
 *-----------------------------------------------------*/

void qr_exit()
{
	printf("\nQR_EXIT: Shutting down QR program ... ");
	/// Sleep to let things shut down properly.
	usleep(200000);

	/// Close the open file descriptors.

	printf("<OK>\n\n");
} /* end qr_exit() */


/*-------------------------------------------------------
 * int main()
 * Initialize data. Open ports. Run main program loop.
 *-----------------------------------------------------*/

int main(int argc, char *argv[])
{
  /// Setup exit function. It is called when SIGINT
  /// (ctrl-c) is invoked.
  void(*exit_ptr)(void);
  exit_ptr = qr_exit;
  atexit(exit_ptr);

	/// Attach signals to exit function.
  struct sigaction sigint_action;
  sigint_action.sa_handler = qr_sigint;
  sigemptyset(&sigint_action.sa_mask);
  sigaddset(&sigint_action.sa_mask, SIGINT);
  sigint_action.sa_flags = 0;
  sigint_action.sa_restorer = NULL;
  sigaction(SIGINT, &sigint_action, NULL);

  /// Declare variables.
  int i = 0, j = 0;
  int done = FALSE;
  double minval = 1.;
  double incval = 1.;
  double maxval = 3.;
  int sleeptime = 0;
  bool b_debug = TRUE;
  matrix<double> *Q;
  matrix<double> *R;

  printf("MAIN: Starting QR ... \n");

  /// Initialize variables.
  Q = new matrix<double>(MAX_STATES, MAX_STATES);
  Q->clear();
  R = new matrix<double>(MAX_SENSORS, MAX_SENSORS);
  R->clear();

  /// Set all elements of Q to the minimum value.
  for (i = 0; i < MAX_STATES; ++i) {
    for (j = 0; j < MAX_STATES; ++j) {
      (*(Q))(i,j) = minval;
    }
  }

  unsigned int ii = 1, jj = 1, idx_row = 1, idx_col = 1;
  while (!done) {
    /// Look at last element to check if it is maxval. If not add incval.
    if ( (*(Q))(Q->size1()-1,Q->size2()-1) < maxval ) {
      printf("MAIN: *** %lf\n", (*(Q))(Q->size1()-1, Q->size2()-1));
      (*(Q))(Q->size1()-1,Q->size2()-1) += incval;
      /// Check progress.
      if (b_debug) {
        //print_matrix(Q, " Q ");
        usleep(sleeptime);
        }
      }
    /// If last element is maxval then go here.
    else {
      printf("MAIN: !!! %lf\n", (*(Q))(Q->size1()-1, Q->size2()-1));
      /// If the first element is not maxval we are not done.
      if ( (*(Q))(0,0) < maxval ) {
        /// Set the last element to minval.
        (*(Q))(0,0) = minval;
        /// Start at the last element and go back through the rows and cols
        /// until reaching the index element, setting each of those elements
        /// to minval.
        for (ii = 1; ii < idx_row; ii++) {
          for (jj = 1; jj < idx_col; jj++) {
            (*(Q))(ii,jj) = minval;
          }
        }
        /// Check if the index element is at maxval.
        if ( (*(Q))(idx_row,idx_col) < maxval ) {
          printf("MAIN: &&& %lf\n", (*(Q))(Q->size1()-1,
          Q->size2()-1));
          /// Increment the index element.
          (*(Q))(idx_row,idx_col) += incval;
          /// Check progress.
          if (b_debug) {
            print_matrix(Q, " Q ");
            usleep(sleeptime);
          }
        }
        else {
          printf("MAIN: --- %lf\n", (*(Q))(Q->size1()-1,
          Q->size2()-1));
          /// Set the index element to minval.
          (*(Q))(idx_row,idx_col) = minval;
          /// Check if the index element is in the first column.
          if (idx_col == 0) {
          /// Set the column index to the last column.
          idx_col = Q->size2()-1;
          /// Move the row index up one row.
          idx_row--;
        }
        else {
          /// Decrement the column index.
          idx_col--;
          }
        }
      }
      /// If last and first elements are maxval then we
      /// are done.
      else {
        done = TRUE;
      }
    }
  }

  return 0;

  } /* end main() */


/*-------------------------------------------------------
* void print_matrix()
* Print out the values of the matrix.
*-----------------------------------------------------*/

void print_matrix(matrix<double> *A,
std::string _NameString)
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
