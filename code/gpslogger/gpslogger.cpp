/*-------------------------------------------------------
 *  Title:        gpslogger.c
 *  Description:  Main program for logging DGPS data.
 *-----------------------------------------------------*/

#include "gpslogger.h"

// Global file descriptors. Only global so that
// gpslogger_exit() can close them.
int gps_fd;
FILE *f_log;

/*-------------------------------------------------------
 * void gpslogger_sigint()
 * Callback for when SIGINT (ctrl-c) is invoked.
 *-----------------------------------------------------*/
void gpslogger_sigint(int signal)
{
  printf("%s(). Caught signal %d ", __FUNCTION__, signal);
  char *s_sig = (char *)"GPSLOGGER_SIGINT";

  psignal(signal, s_sig);

  exit(0);
} /* end gpslogger_sigint() */

/*-------------------------------------------------------
 * void gpslogger_exit()
 * Exit function for main program.
 *-----------------------------------------------------*/
void gpslogger_exit()
{
  printf("\n%s(). Shutting down gpslogger
         program ... ", __FUNCTION__);

  // Sleep to let things shut down properly.
  usleep(200000);

  // Close the open file descriptors.
  if (gps_fd > 0)
  {
    char *sOEM4_unlogall;
    sOEM4_unlogall = (char *)"\runlogall\r";
    serial_send(gps_fd, sOEM4_unlogall, strlen(sOEM4_unlogall));
    close(gps_fd);
  }

  if (f_log > 0)
  {
    fclose(f_log);
  }

  printf("<OK>\n\n");
} /* end gpslogger_exit() */

/*-------------------------------------------------------
 * int main()
 * Run main program loop.
 *-----------------------------------------------------*/

int main(int argc, char *argv[])
{
  // Setup exit function
  void(*exit_ptr)(void);
  exit_ptr = gpslogger_exit;
  atexit(exit_ptr);

  // Attach signals to exit function.
  struct sigaction sigint_action;
  sigint_action.sa_handler = gpslogger_sigint;
  sigemptyset(&sigint_action.sa_mask);
  sigaddset(&sigint_action.sa_mask, SIGINT);
  sigint_action.sa_flags = 0;
  sigint_action.sa_restorer = NULL;
  sigaction(SIGINT, &sigint_action, NULL);

  // Declare variables.
  int status = 0;
  int msg_bytes = 0;
  int mode = STAND_ALONE;
  int gps_baud = 115200;
  int got_position = FALSE;
  char buf[SERIAL_MAX_DATA];
  char *p_buf = &buf[0];
  char oem4_str[OEM4_STRING_SIZE];
  char *oem4_cmd = &oem4_str[0];
  CONF_VARS cf;
  BESTPOS pos;
  TIMING timer_print;
  TIMING timer_log;
  TIMING timer_base;

  printf("MAIN: Starting gpslogger ... \n");

  // Initialize variables.
  gps_fd = 0;
  f_log = NULL;
  memset(&buf, 0, SERIAL_MAX_DATA);
  memset(&cf, 0, sizeof(CONF_VARS));
  memset(&pos, 0, sizeof(BESTPOS));

  // Initialize timers.
  timing_set_timer(&timer_print);
  timing_set_timer(&timer_log);
  timing_set_timer(&timer_base);

  // Parse command line arguments.
  parse_default_config(&cf);
  parse_cla(argc, argv, &cf,
            (const char *)GPSLOGGER_FILENAME);
  mode = cf.op_mode;

  // Set up the GPS receiver.
  if (cf.enable_gps)
  {
    gps_fd = serial_setup(cf.gps_port, cf.gps_baud);
    if (gps_fd > 0)
    {
      // Make sure no messages are being output by the GPS receiver.
      if (oem4_unlogall(gps_fd))
      {
        oem4_log_bestpos(gps_fd, cf.period_gps, (char *)"com1");
      }
    }
  }

  // Change the GPS receiver baud rate.
  if (gps_fd > 0)
  {
    if (oem4_set_baud(gps_fd, gps_baud, (char *)"com1"))
    {
      sleep(1);
      if ((gps_fd = serial_setup(cf.gps_port, gps_baud)))
      {
        printf("<OK>\n");
      }
      else
      {
        printf("FAILED\n");
      }
    }
    else
    {
      printf("FAILED\n");
    }
  }

  // Configure the GPS receiver in STAND_ALONE mode to
  // accept RTCA DGPS corrections.
  if (cf.enable_gps)
  {
    snprintf(oem4_cmd, OEM4_STRING_SIZE, "\rinterfacemode
             com2 rtca none on\r");
    status = serial_send(gps_fd, oem4_cmd, strlen(oem4_cmd));
  }

  // Set up a log file.
  if (cf.enable_log)
  {
    f_log = fopen("logs/log.csv", "w");
    if (f_log)
    {
      printf("MAIN: Creating log file OK.\n");
    }
    else
    {
      printf("MAIN: Creating log file FAILED.\n");
    }
  }

  printf("MAIN: gpslogger running now.\n");

  // Main loop. Will exit on <ctrl-c>.
  while (1)
  {
    // Get GPS data.
    if ((cf.enable_gps) && (gps_fd > 0))
    {
      // Get new serial data and store in buffer.
      msg_bytes = oem4_get_data(gps_fd, buf, p_buf, &pos);

      // Keep track of position in buffer.
      p_buf = &buf[0];
      p_buf += strlen(buf);
    }

    // Log data if flag is set and data is available.
    if (cf.enable_log && f_log && msg_bytes)
    {
      status = oem4_write_logfile(f_log, &pos);
    }

    // Print data.
    if (timing_check_period(&timer_print, 1))
    {
      timing_set_timer(&timer_print);
      oem4_print_status(&pos);
    }

    // Reset the message found variable.
    msg_bytes = FALSE;
  }

  exit(0);
} /* end main() */
