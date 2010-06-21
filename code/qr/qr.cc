/*--------------------------------------------------------------------------
 *  Title:        qr.cc
 *  Description:  Main program for training the Q and R covariance matrices.
 *------------------------------------------------------------------------*/

#include "qr.h"

/*--------------------------------------------------------------------------
 * void print_matrix()
 * Print out the values of the matrix.
 *------------------------------------------------------------------------*/
void QR::print_matrix(matrix<double> *A, std::string _NameString)
{
  unsigned int i = 0, j = 0;

  fprintf(stderr, "\nQR::%s(). %s with size <%d, %d> is:\n( "
    , __FUNCTION__, _NameString.c_str(), (int)A->size1(), (int)A->size2());
  for (i = 0; i < A->size1(); i++)
  {
    for (j = 0; j < A->size2(); j++)
    {
      fprintf(stderr, "%.15f ", (*(A))(i, j));
      if ((j == (A->size2() - 1)) && (i != (A->size1() - 1)))
      {
        fprintf(stderr, ")\n( ");
      }
    }
  }
  fprintf(stderr, ")\n");

  return;
} /* end print_matrix() */

/*--------------------------------------------------------------------------
 * void print_matrix_file()
 * Print the values of the matrix to a file.
 *------------------------------------------------------------------------*/
void QR::print_matrix_file(matrix<double> *A, std::string _NameString,
    FILE *f_qropt)
{
  unsigned int i = 0, j = 0;

  fprintf(f_qropt, "\n%s:\n( ", _NameString.c_str());
  for (i = 0; i < A->size1(); i++)
  {
    for (j = 0; j < A->size2(); j++)
    {
      fprintf(f_qropt, "%.15f ", (*(A))(i, j));
      if ((j == (A->size2() - 1)) && (i != (A->size1() - 1)))
      {
        fprintf(f_qropt, ")\n( ");
      }
    }
  }
  fprintf(f_qropt, ")\n");

  return;
} /* end print_matrix() */

/*--------------------------------------------------------------------------
 * void print_progress()
 * Print the progress made through the permutations.
 *------------------------------------------------------------------------*/
void QR::print_progress(int count, float *progress)
{
  // Declare variables.
  float num_permutations = pow(2., pow(m_iNumStates, 2.)) *
        pow(2., pow(m_iNumSensors, 2.));
  float current_progress = 0.;
  float i = 0.;

  current_progress = count / num_permutations * 100.;
  for (i = 100.; i > 0.; i -= .1)
  {
    if (current_progress >= i && *progress < i)
    {
      printf("\r%.1f%%", current_progress);
      break;
    }
  }
  printf("\r%.1f%%", current_progress);

  if (count == num_permutations)
  {
    printf("\n");
  }

  *progress = current_progress;

  return;
} /* end print_progress() */

/*--------------------------------------------------------------------------
 * void init_matrix()
 * Initializes A matrix to minval back from the last element to an index
 * element.
 *------------------------------------------------------------------------*/
void QR::init_matrix(matrix<double> *A, double minval, int idx_row,
        int idx_col)
{
  // Declare variables.
  int i = 0, j = 0;

  if (m_bDiagonalOnly)
  {
    // Set all diagonal elements of A from the index row and below
    // to the minimum value.
    for (i = A->size1() - 1; i > idx_row - 1; i--)
    {
      for (j = A->size2() - 1; j > idx_col - 1; j--)
      {
        if (i == j)
        {
          (*(A))(i, j) = minval;
        }
      }
    }
  }
  else
  {
    // Set all elements of A from the index row and below
    // to the minimum value.
    for (i = A->size1() - 1; i > idx_row; i--)
    {
      for (j = A->size2() - 1; j > -1; j--)
      {
        (*(A))(i, j) = minval;
      }
    }

    // Set all elements of A from the last to the index column
    // to the minimum value.
    for (j = A->size2() - 1; j > idx_col - 1; j--)
    {
      (*(A))(idx_row, j) = minval;
    }
  }

  return;
} /* end init_matrix() */

/*--------------------------------------------------------------------------
 * void init_Q()
 * Initializes Q matrix to minval back from the last element to an index
 * element.
 *------------------------------------------------------------------------*/
void QR::init_Q(int idx_row, int idx_col)
{
  // Declare variables.
  int i = 0, j = 0;

  // Set all elements of Q from the index row and below
  // to the optimal value.
  for (i = Q->size1() - 1; i > idx_row; i--)
  {
    for (j = Q->size2() - 1; j > -1; j--)
    {
      (*(Q))(i, j) = (*(Qopt))(i, j) - (*(Qinc))(i, j);
    }
  }

  // Set all elements of Q from the last to the index column
  // to the optimal value.
  for (j = Q->size2() - 1; j > idx_col - 1; j--)
  {
    (*(Q))(idx_row, j) = (*(Qopt))(idx_row, j) - (*(Qinc))(i, j);
  }

  return;
} /* end init_Q() */

/*--------------------------------------------------------------------------
 * void init_R()
 * Initializes R matrix to minval back from the last element to an index
 * element.
 *------------------------------------------------------------------------*/
void QR::init_R(int idx_row, int idx_col)
{
  // Declare variables.
  int i = 0, j = 0;

  // Set all elements of R from the index row and below
  // to the optimal value.
  for (i = R->size1() - 1; i > idx_row; i--)
  {
    for (j = R->size2() - 1; j > -1; j--)
    {
      (*(R))(i, j) = (*(Ropt))(i, j) - (*(Rinc))(i, j);
    }
  }

  // Set all elements of R from the last to the index column
  // to the optimal value.
  for (j = R->size2() - 1; j > idx_col - 1; j--)
  {
    (*(R))(idx_row, j) = (*(Ropt))(idx_row, j) - (*(Rinc))(i, j);
  }

  return;
} /* end init_R() */

/*--------------------------------------------------------------------------
 * int find_nonmax_Q()
 * Goes from last to first element looking for element that is not maxval.
 *------------------------------------------------------------------------*/
int QR::find_nonmax_Q(int *nonmax_row, int *nonmax_col)
{
  // Declare variables.
  int i = 0, j = 0;

  // Set all elements of Q to the minimum value.
  for (i = Q->size1() - 1; i > -1; i--)
  {
    for (j = Q->size2() - 1; j > -1; j--)
    {
      if (m_bDiagonalOnly)
      {
        if (i == j)
        {
          if ((*(Q))(i, j) < (*(Qopt))(i, j) + (*(Qinc))(i, j))
          {
            *nonmax_row = i;
            *nonmax_col = j;
            return TRUE;
          }
        }
      }
      else
      {
        if ((*(Q))(i, j) < (*(Qopt))(i, j) + (*(Qinc))(i, j))
        {
          *nonmax_row = i;
          *nonmax_col = j;
          return TRUE;
        }
      }
    }
  }

  return FALSE;
} /* end find_nonmax_Q() */

/*--------------------------------------------------------------------------
 * int find_nonmax_R()
 * Goes from last to first element looking for element that is not maxval.
 *------------------------------------------------------------------------*/
int QR::find_nonmax_R(int *nonmax_row, int *nonmax_col)
{
  // Declare variables.
  int i = 0, j = 0;

  // Set all elements of Q to the minimum value.
  for (i = R->size1() - 1; i > -1; i--)
  {
    for (j = R->size2() - 1; j > -1; j--)
    {
      if (m_bDiagonalOnly)
      {
        if (i == j)
        {
          if ((*(R))(i, j) < (*(Ropt))(i, j) + (*(Rinc))(i, j))
          {
            *nonmax_row = i;
            *nonmax_col = j;
            return TRUE;
          }
        }
      }
      else
      {
        if ((*(R))(i, j) < (*(Ropt))(i, j) + (*(Rinc))(i, j))
        {
          *nonmax_row = i;
          *nonmax_col = j;
          return TRUE;
        }
      }
    }
  }

  return FALSE;
} /* end find_nonmax_R() */

/*--------------------------------------------------------------------------
 * void q()
 * Generates all possible Q matrices.
 *------------------------------------------------------------------------*/
void QR::q(PARSE_GPS_VALS *kf, PARSE_GPS_VALS *gt, int length)
{
  // Declare variables.
  int nonmax_remaining = TRUE;
  int done = FALSE;
  int nonmax_row = Q->size1() - 1;
  int nonmax_col = Q->size2() - 1;
  int count = 0;
  double tmp = 0.;

  // Set all elements of Q to the minimum value.
  init_Q(0, 0);
  // For this Q try all possible R matrices.
  r(kf, gt, length);

  while (!done)
  {
    // Look for last element that is less than maxval.
    nonmax_remaining = find_nonmax_Q(&nonmax_row, &nonmax_col);
    if (m_bSearchPrevOptimal)
    {
      (*(Q))(nonmax_row, nonmax_col) += (*(Qinc))(nonmax_row, nonmax_col);
    }
    else
    {
      (*(Q))(nonmax_row, nonmax_col) +=
		  2 * (*(Qinc))(nonmax_row, nonmax_col);
    }
    count++;
    // For this Q try all possible R matrices.
    count += r(kf, gt, length);

    if ((*(Q))(nonmax_row, nonmax_col) >= (*(Qopt))(nonmax_row, nonmax_col)
		  + (*(Qinc))(nonmax_row, nonmax_col))
    {
      // Look for last element that is less than maxval.
      nonmax_remaining = find_nonmax_Q(&nonmax_row, &nonmax_col);
      if (!nonmax_remaining)
      {
        done = TRUE;
      }
      // Save the value of the last nonmax element.
      tmp = (*(Q))(nonmax_row, nonmax_col);
      // Set all elements to minval except for the index element.
      init_Q(nonmax_row, nonmax_col);
      if (m_bSearchPrevOptimal)
      {
        (*(Q))(nonmax_row, nonmax_col) = tmp +
                (*(Qinc))(nonmax_row, nonmax_col);
      }
      else
      {
        (*(Q))(nonmax_row, nonmax_col) = tmp
                + 2 * (*(Qinc))(nonmax_row, nonmax_col);
      }
      count++;
      // For this Q try all possible R matrices.
      count += r(kf, gt, length);
    }
  }
} /* end q() */

/*--------------------------------------------------------------------------
 * int r()
 * Generates all possible R matrices.
 *------------------------------------------------------------------------*/
int QR::r(PARSE_GPS_VALS *kf, PARSE_GPS_VALS *gt, int length)
{
  // Declare variables.
  int nonmax_remaining = TRUE;
  int count = 0;
  int done = FALSE;
  int nonmax_row = R->size1() - 1;
  int nonmax_col = R->size2() - 1;
  double tmp = 0.;

  // Set all elements of R to the current optimal value and compute error.
  init_R(0, 0);

  // Write current optimal <Q,R> to the KF params file.
  write_kf_params();

  // Fake getting new data from KF simulation.
  parse_gps_simulate(kf, gt, PARSE_GPS_MAX_VALS);

  // Calculate the distance error between KF and DGPS output.
  m_dError = metrics_residual(length, kf->lat, kf->lon, gt->lat, gt->lon);
  if (m_dError < m_dErrorMin)
  {
    // Found a minimum error so record error and <Q,R> that generated it.
    m_bMinFound = TRUE;
    m_dErrorMin = m_dError;
    *RoptHold = *R;
    *QoptHold = *Q;
  }

  while (!done)
  {
    // Look for last element that is less than maxval.
    nonmax_remaining = find_nonmax_R(&nonmax_row, &nonmax_col);

    // Increment the last nonmax element and compute the error.
    if (m_bSearchPrevOptimal)
    {
      (*(R))(nonmax_row, nonmax_col) += (*(Rinc))(nonmax_row, nonmax_col);
    }
    else
    {
      (*(R))(nonmax_row, nonmax_col) += 2 *
		  (*(Rinc))(nonmax_row, nonmax_col);
    }
    count++;

    // Write current optimal <Q,R> to the KF params file used by the ACS KF.
    write_kf_params();

    // Fake getting new data from KF simulation.
    parse_gps_simulate(kf, gt, PARSE_GPS_MAX_VALS);

    // Calculate the distance error between KF and DGPS output.
    m_dError = metrics_residual(length, kf->lat, kf->lon, gt->lat, gt->lon);
    if (m_dError < m_dErrorMin)
    {
      // Found a minimum error so record error and <Q,R> that generated it.
      m_bMinFound = TRUE;
      m_dErrorMin = m_dError;
      *RoptHold = *R;
      *QoptHold = *Q;
    }

    if ((*(R))(nonmax_row, nonmax_col) >= (*(Ropt))(nonmax_row, nonmax_col)
            + (*(Rinc))(nonmax_row, nonmax_col))
    {
      // Look for last element that is less than maxval.
      nonmax_remaining = find_nonmax_R(&nonmax_row, &nonmax_col);
      if (!nonmax_remaining)
      {
        done = TRUE;
        break;
      }
      // Save the value of the last nonmax element.
      tmp = (*(R))(nonmax_row, nonmax_col);

      // Update R and compute the error.
      init_R(nonmax_row, nonmax_col);
      if (m_bSearchPrevOptimal)
      {
        (*(R))(nonmax_row, nonmax_col) = tmp +
            (*(Rinc))(nonmax_row, nonmax_col);
      }
      else
      {
        (*(R))(nonmax_row, nonmax_col) = tmp
                + 2 * (*(Rinc))(nonmax_row, nonmax_col);
      }
      count++;

      // Write current optimal <Q,R> to KF params file used by the ACS KF.
      write_kf_params();

      // Fake getting new data from KF simulation.
      parse_gps_simulate(kf, gt, PARSE_GPS_MAX_VALS);

      // Calculate the distance error between KF and DGPS output.
      m_dError = metrics_residual(length,
								kf->lat, kf->lon, gt->lat, gt->lon);
      if (m_dError < m_dErrorMin)
      {
        // Found a minimum error so record error and <Q,R> that generated it.
        m_bMinFound = TRUE;
        m_dErrorMin = m_dError;
        *RoptHold = *R;
        *QoptHold = *Q;
      }
    }
  }

  return count;
} /* end r() */

/*--------------------------------------------------------------------------
 * void Setup()
 * Initializes the variables in this class. Starts up ACS modules.
 *------------------------------------------------------------------------*/
void QR::Setup()
{
  // Initialize variables.
  m_iDebug = 1;
  m_bDiagonalOnly = TRUE;
  m_bSearchPrevOptimal = FALSE;
  m_iNumStates = 8;
  m_iNumSensors = 5;
  m_dMinval = 1.;
  m_dMaxval = 3.;
  //m_dDivisor = m_dMaxval - m_dMinval;
  m_dDivisor = 1.;
  m_dError = HUGE_VAL;
  m_dErrorMin = HUGE_VAL;
  m_bMinFound = FALSE;
  m_dIncPercent = 0.5;

  // Create and clear matrices.
  Q = new matrix<double>(m_iNumStates, m_iNumStates);
  R = new matrix<double>(m_iNumSensors, m_iNumSensors);
  Qopt = new matrix<double>(m_iNumStates, m_iNumStates);
  Ropt = new matrix<double>(m_iNumSensors, m_iNumSensors);
  Qinc = new matrix<double>(m_iNumStates, m_iNumStates);
  Rinc = new matrix<double>(m_iNumSensors, m_iNumSensors);
  QoptHold = new matrix<double>(m_iNumStates, m_iNumStates);
  RoptHold = new matrix<double>(m_iNumSensors, m_iNumSensors);
  Q->clear();
  R->clear();
  Qopt->clear();
  Ropt->clear();
  Qinc->clear();
  Rinc->clear();
  QoptHold->clear();
  RoptHold->clear();

  // Initialize the matrices based on grid size.
  init_matrix(Q, m_dMaxval - (m_dDivisor / 2.), 0, 0);
  init_matrix(R, m_dMaxval - (m_dDivisor / 2.), 0, 0);
  init_matrix(Qopt, m_dMaxval - (m_dDivisor / 2.), 0, 0);
  init_matrix(Ropt, m_dMaxval - (m_dDivisor / 2.), 0, 0);
  init_matrix(Qinc, m_dDivisor / 2., 0, 0);
  init_matrix(Rinc, m_dDivisor / 2., 0, 0);

  // Start up the ACS components.
  //start_acs();

  fprintf(stderr, "QR::%s(). Finished.\n", __FUNCTION__);
} /* end Setup() */

/*--------------------------------------------------------------------------
 * void recalculate_incs()
 * Calculate the new increment matrices for the finer grid spacing.
 *------------------------------------------------------------------------*/
void QR::recalculate_incs()
{
  int i = 0, j = 0;

  for (i = Qinc->size1() - 1; i > -1; i--)
  {
    for (j = Qinc->size2() - 1; j > -1; j--)
    {
      (*(Qinc))(i, j) /= 2.;
    }
  }

  i = 0;
  j = 0;

  for (i = Rinc->size1() - 1; i > -1; i--)
  {
    for (j = Rinc->size2() - 1; j > -1; j--)
    {
      (*(Rinc))(i, j) /= 2.;
    }
  }
} /* end recalculate_incs() */

/*--------------------------------------------------------------------------
 * int write_kf_params()
 * Write the current <Q,R> matrices to a file that the ACS KF can read.
 *------------------------------------------------------------------------*/
int QR::write_kf_params()
{
  // Declare variables.
  FILE *fd = NULL;

  // Open the file.
  fd = fopen(m_sKFParamsFilename, "w");
  if (fd == NULL)
  {
    fprintf(stderr, "QR::%s(). WARNING!!! Error opening config "
            "file %s\n", __FUNCTION__, m_sKFParamsFilename);
    return FALSE;
  }

  // Write the diagonal elements of <Q,R> to the open file.
  fprintf(
    fd,
    "<Q_X> %.12lf </Q_X>\n"
    "<Q_Y> %.12lf </Q_Y>\n"
    "<Q_Z> %.12lf </Q_Z>\n"
    "<Q_VELOCITY> %.12lf </Q_VELOCITY>\n"
    "<Q_PITCH> %.12lf </Q_PITCH>\n"
    "<Q_ROLL> %.12lf </Q_ROLL>\n"
    "<Q_YAW> %.12lf </Q_YAW>\n"
    "<Q_YAWSPEED> %.12lf </Q_YAWSPEED>\n"
    "<TOTAL_TIME_RUN_ON_ROBOT> 0.00 </TOTAL_TIME_RUN_ON_ROBOT>\n",
    (*(Q))(0, 0), (*(Q))(1, 1), (*(Q))(2, 2), (*(Q))(3, 3),
    (*(Q))(4, 4), (*(Q))(5, 5), (*(Q))(6, 6), (*(Q))(7, 7)
  );

  // Close the file.
  fclose(fd);

  return TRUE;
} /* end write_kf_params() */

/*--------------------------------------------------------------------------
 * int read_kf_params_line()
 * Read and parse a line from the initial KF parameters.
 *------------------------------------------------------------------------*/
double QR::read_kf_params_line(char *buf)
{
  // Declare variables.
  int i = 0;
  char *token;
  char *saveptr;
  char *delim = (char *)" <>";
  char *tokens[5];

  // Initialize variables.
  memset(tokens, 0, sizeof(tokens));

  // Split the incoming buffer up into tokens.
  for (i = 0; ; i++, buf = NULL)
  {
    token = strtok_r(buf, delim, &saveptr);
    if (token == NULL)
    {
      break;
    }
    tokens[i] = token;
  }

  return strtod(tokens[1], NULL);
} /* end read_kf_params_line() */

/*--------------------------------------------------------------------------
 * int read_kf_params()
 * Read the initial KF params.
 *------------------------------------------------------------------------*/
int QR::read_kf_params()
{
  // Declare variables.
  FILE *fd = NULL;
  int lines = 0;
  int i = 0;
  char c;
  char buf[10000];
  double param_val = 0.;

  // Initialize variables.
  memset(&buf, 0, sizeof(buf));

  // Open the file.
  fd = fopen(m_sKFInitParamsFilename, "r");
  if (fd == NULL)
  {
    fprintf(stderr, "QR::%s(). Error opening config file %s\n"
        , __FUNCTION__, m_sKFInitParamsFilename);
    return FALSE;
  }

  // Determine the number of lines in the file.
  while ((c = fgetc(fd)) != EOF)
  {
    if (c == '\n')
    {
      lines++;
    }
  }

  // Reset the file pointer to the beginning of the file.
  fseek(fd, 0, SEEK_SET);

  // Read one line at a time and parse it.
  while (fgets(buf, sizeof(buf), fd) != NULL)
  {
    param_val = read_kf_params_line(buf);
    switch (i)
    {
    case 0:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 1:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 2:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 3:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 4:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 5:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 6:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 7:
      (*(Q))(i, i) = param_val;
      (*(Qopt))(i, i) = param_val;
      (*(Qinc))(i, i) = param_val * m_dIncPercent;
      break;

    case 8:
      break;

    case 9:
      (*(R))(i - 9, i - 9) = param_val;
      (*(Ropt))(i - 9, i - 9) = param_val;
      (*(Rinc))(i - 9, i - 9) = param_val * m_dIncPercent;
      break;

    case 10:
      (*(R))(i - 9, i - 9) = param_val;
      (*(Ropt))(i - 9, i - 9) = param_val;
      (*(Rinc))(i - 9, i - 9) = param_val * m_dIncPercent;
      break;

    case 11:
      (*(R))(i - 9, i - 9) = param_val;
      (*(Ropt))(i - 9, i - 9) = param_val;
      (*(Rinc))(i - 9, i - 9) = param_val * m_dIncPercent;
      break;

    case 12:
      (*(R))(i - 9, i - 9) = param_val;
      (*(Ropt))(i - 9, i - 9) = param_val;
      (*(Rinc))(i - 9, i - 9) = param_val * m_dIncPercent;
      break;

    case 13:
      (*(R))(i - 9, i - 9) = param_val;
      (*(Ropt))(i - 9, i - 9) = param_val;
      (*(Rinc))(i - 9, i - 9) = param_val * m_dIncPercent;
      break;

    default:
      break;
    }
    i++;
  }

  // Close the file.
  fclose(fd);

  return TRUE;
} /* end read_kf_params() */

/*--------------------------------------------------------------------------
 * QR()
 * Constructor.
 *------------------------------------------------------------------------*/
QR::QR()
{
  Setup();
} /* end QR() */

/*--------------------------------------------------------------------------
 * ~QR()
 * Destructor.
 *------------------------------------------------------------------------*/
QR::~QR()
{
  delete Q;
  delete R;
  delete Qopt;
  delete Ropt;
  delete Qinc;
  delete Rinc;
  delete QoptHold;
  delete RoptHold;
} /* end ~QR() */

/*--------------------------------------------------------------------------
 * run()
 * Run through the algorithm.
 *------------------------------------------------------------------------*/
void QR::run(PARSE_GPS_VALS *kf, PARSE_GPS_VALS *gt, int length)
{
  m_dDivisor /= 2.;
  recalculate_incs();
  q(kf, gt, length);
} /* end run() */

/*--------------------------------------------------------------------------
 * start_acs()
 * Start the ACS components.
 *------------------------------------------------------------------------*/
void QR::start_acs()
{
  // Read configuration files and set appropriate variables.
  ConfigParserPtr m_pConfigParser = ACSBase::getConfigParserPtr();
  KalmanLocal *m_pKL;
  if (!m_pConfigParser->readFile("./Config/Robot.xml"))
  {
    exit(0);
  }
  else
  {
    if (!(m_pConfigParser->parse_file()))
    {
      exit(0);
    }
    // Make sure to set up the Kalman filter _before_ the Data Playback.
    // That way no data will be missed by the KF.
    // Create the ACS Kalman filter and initialize the states.
    m_pKL = new KalmanLocal("qr");
    m_pKL->setState(ACS_X_STATE, 0.0);
    m_pKL->setState(ACS_Y_STATE, 0.0);
    m_pKL->setState(ACS_Z_STATE, 0.0);
    m_pKL->setState(ACS_VELOCITY_STATE, 0.0);
    m_pKL->setState(ACS_PITCH_STATE, 0.0);
    m_pKL->setState(ACS_ROLL_STATE, 0.0);
    m_pKL->setState(ACS_YAW_STATE, 0.0);
    m_pKL->setState(ACS_YAW_SPEED_STATE, 0.0);
    // Let the Kalman filter start up.
    usleep(100000);

    // Set up ACS data playback.
    ACSDataPlayback *m_pDP = new ACSDataPlayback("packbotPlayback");

    // Start the Kalman filter thread.
    m_pKL->setupFromDataContainer(
      ACSDataContainer(
        "QR",
        "QR",
        ACS_SRC_COMMS,
        ACS_MSG_STRING,
        ACS_MSG_SUB_CMD_WAKEUP,
        "Any",
        "Any",
        ACS_SRC_KALMAN_FILTER,
        ACS_MSG_FORMAT_OBJECT,
        ACSString("Wakeup", "QR")
      )
    );

    m_pDP->Setup();
  }
} /* end start_acs() */
