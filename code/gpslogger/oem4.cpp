/*-------------------------------------------------------
 *  Title:        oem4.c
 *  Description:  File to handle Novatel OEM4 GPS data.
 *-----------------------------------------------------*/

#include "oem4.h"

/*-------------------------------------------------------
 * int oem4_get_data()
 * Get data from OEM4.
 *-----------------------------------------------------*/
int oem4_get_data(int fd, char *buf, char *p_buf, BESTPOS *pos)
{
  // Declare variables.
  int recv_bytes = 0;
  int bytes_to_read = 0;
  int msg_bytes = 0;
  char parsebuf[SERIAL_MAX_DATA];

  // Initialize variables.
  memset(&parsebuf, 0, SERIAL_MAX_DATA);

  // Get new serial data using pointer and store in buffer.
  bytes_to_read = serial_bytes_available(fd);
  if (bytes_to_read)
  {
    recv_bytes = serial_recv(fd, p_buf, bytes_to_read);

    // Move p_buf to the end of buf.
    p_buf += recv_bytes;

    // Parse buf and get back position of beginning of last incomplete message.
    memcpy(parsebuf, buf, SERIAL_MAX_DATA);
    msg_bytes = oem4_parse_string(parsebuf, pos);

    // Move the buffer to the beginning of the incomplete message.
    memmove(buf, &buf[msg_bytes], SERIAL_MAX_DATA - msg_bytes);
  }

  return msg_bytes;
} /* end oem4_get_data() */


/*-------------------------------------------------------
 * int oem4_parse_string()
 * Parses data from OEM4 return messages.
 *-----------------------------------------------------*/
int oem4_parse_string(char *buf, BESTPOS *pos)
{
  // Declare variables.
  int ii = 0;
  int retval = 0;
  int byte_count = 0;
  int tmp_start = 0;
  int msg_start = 0;
  int msg_found = FALSE;
  int msg_found_start = FALSE;
  int msg_found_end = FALSE;
  char *token;
  char *saveptr;
  char *delim = (char *)OEM4_DELIM;
  char *tokens[OEM4_STRING_SIZE];

  // Initialize variables.
  memset(tokens, 0, sizeof(tokens));

  // Split the incoming buffer up into tokens based on the delimiting characters.
  for (ii = 0; ; ii++, buf = NULL)
  {
    token = strtok_r(buf, delim, &saveptr);
    if (token == NULL)
    {
      break;
    }
    tokens[ii] = token;
    byte_count += strlen(token) + 1;

    // Look for start of BESTPOS message. This is only a temporary start index because the entire message is not
    // necessarily in the buffer. We will only fill in the BESTPOS struct with data from the last complete message in the buffer.
    if (strncmp(token, OEM4_BESTPOS_START, strlen(OEM4_BESTPOS_START)) == 0)
    {
      tmp_start = ii;
      msg_found_start = TRUE;
      msg_found_end = FALSE;
    }

    // Have to find two OEM4_BESTPOS_END tokens per one OEM4_BESTPOS_START token for a complete message.
    if (strncmp(token, OEM4_BESTPOS_END, strlen(OEM4_BESTPOS_END)) == 0)
    {
      if (msg_found_end)
      {
        if (strncmp(token, OEM4_BESTPOS_END, strlen(OEM4_BESTPOS_END)) == 0)
        {
          // Found entire message after tmp_start so that really is the start of a complete message that can be parsed.
          msg_start = tmp_start;
          msg_found = TRUE;
          msg_found_start = FALSE;
          msg_found_end = FALSE;
          retval += byte_count;
        }
        else
        {
          // No message found so return FALSE.
          return FALSE;
        }
      }
      msg_found_end = TRUE;
    }
  }

  // Found a full message so go parse it.
  if (msg_found)
  {
    oem4_parse_bestpos(tokens, pos, msg_start);
  }

  return retval;
} /* end oem4_parse_string() */


/*-------------------------------------------------------
 * int oem4_parse_bestpos()
 * Parses a BESTPOS OEM4 message.
 *-----------------------------------------------------*/
int oem4_parse_bestpos(char **tokens, BESTPOS *pos, int idx)
{
  // Check solution status.
  idx += 10;
  if (tokens[idx] == NULL)
  {
    pos->status = INSUFFICIENT_OBS;
  }
  else if (strncmp(tokens[idx], "SOL_COMPUTED", strlen("SOL_COMPUTED")) == 0)
  {
    pos->status = SOL_COMPUTED;
  }
  else if (strncmp(tokens[idx], "INSUFFICIENT_OBS", strlen("INSUFFICIENT_OBS")) == 0)
  {
    pos->status = INSUFFICIENT_OBS;
  }
  else
  {
    pos->status = INSUFFICIENT_OBS;
  }
  idx++;

  // Check position type.
  if (tokens[idx] == NULL)
  {
    pos->type = NONE;
  }
  else if (strncmp(tokens[idx], "NONE", strlen("NONE")) == 0)
  {
    pos->type = NONE;
  }
  else if (strncmp(tokens[idx], "FIXEDPOS", strlen("FIXEDPOS")) == 0)
  {
    pos->type = FIXEDPOS;
  }
  else if (strncmp(tokens[idx], "FIXED_HEIGHT", strlen("FIXED_HEIGHT")) == 0)
  {
    pos->type = FIXED_HEIGHT;
  }
  else if (strncmp(tokens[idx], "FIXEDVEL", strlen("FIXEDVEL")) == 0)
  {
    pos->type = FIXEDVEL;
  }
  else if (strncmp(tokens[idx], "SINGLE", strlen("SINGLE")) == 0)
  {
    pos->type = SINGLE;
  }
  else if (strncmp(tokens[idx], "PSRDIFF", strlen("PSRDIFF")) == 0)
  {
    pos->type = PSRDIFF;
  }
  else if (strncmp(tokens[idx], "WAAS", strlen("WAAS")) == 0)
  {
    pos->type = WAAS;
  }
  else if (strncmp(tokens[idx], "L1_FLOAT", strlen("L1_FLOAT")) == 0)
  {
    pos->type = L1_FLOAT;
  }
  else if (strncmp(tokens[idx], "IONOFREE_FLOAT", strlen("IONOFREE_FLOAT")) == 0)
  {
    pos->type = IONOFREE_FLOAT;
  }
  else if (strncmp(tokens[idx], "NARROW_FLOAT", strlen("NARROW_FLOAT")) == 0)
  {
    pos->type = NARROW_FLOAT;
  }
  else if (strncmp(tokens[idx], "L1_INT", strlen("L1_INT")) == 0)
  {
    pos->type = L1_INT;
  }
  else if (strncmp(tokens[idx], "WIDE_INT", strlen("WIDE_INT")) == 0)
  {
    pos->type = WIDE_INT;
  }
  else if (strncmp(tokens[idx], "NARROW_INT", strlen("NARROW_INT")) == 0)
  {
    pos->type = NARROW_INT;
  }
  else
  {
    pos->type = NONE;
  }
  idx++;

  // Check latitude.
  pos->lat = atof(tokens[idx]);
  idx++;

  // Check longitude.
  pos->lon = atof(tokens[idx]);
  idx++;

  // Check height.
  pos->hgt = atof(tokens[idx]);
  idx++;

  // Check undulation.
  pos->undulation = atof(tokens[idx]);
  idx++;

  // Check datumid.
  if (tokens[idx] == NULL)
  {
    pos->datumid = WGS84;
  }
  else if (strncmp(tokens[idx], "ADIND", strlen("ADIND")) == 0)
  {
    pos->datumid = ADIND;
  }
  else if (strncmp(tokens[idx], "ARC50", strlen("ARC50")) == 0)
  {
    pos->datumid = ARC50;
  }
  else if (strncmp(tokens[idx], "ARC60", strlen("ARC60")) == 0)
  {
    pos->datumid = ARC60;
  }
  else if (strncmp(tokens[idx], "AGD66", strlen("AGD66")) == 0)
  {
    pos->datumid = AGD66;
  }
  else if (strncmp(tokens[idx], "AGD84", strlen("AGD84")) == 0)
  {
    pos->datumid = AGD84;
  }
  else if (strncmp(tokens[idx], "BUKIT", strlen("BUKIT")) == 0)
  {
    pos->datumid = BUKIT;
  }
  else if (strncmp(tokens[idx], "ASTRO", strlen("ASTRO")) == 0)
  {
    pos->datumid = ASTRO;
  }
  else if (strncmp(tokens[idx], "CHATM", strlen("CHATM")) == 0)
  {
    pos->datumid = CHATM;
  }
  else if (strncmp(tokens[idx], "CARTH", strlen("CARTH")) == 0)
  {
    pos->datumid = CARTH;
  }
  else if (strncmp(tokens[idx], "CAPE", strlen("CAPE")) == 0)
  {
    pos->datumid = CAPE;
  }
  else if (strncmp(tokens[idx], "DJAKA", strlen("DJAKA")) == 0)
  {
    pos->datumid = DJAKA;
  }
  else if (strncmp(tokens[idx], "EGYPT", strlen("EGYPT")) == 0)
  {
    pos->datumid = EGYPT;
  }
  else if (strncmp(tokens[idx], "ED50", strlen("ED50")) == 0)
  {
    pos->datumid = ED50;
  }
  else if (strncmp(tokens[idx], "ED79", strlen("ED79")) == 0)
  {
    pos->datumid = ED79;
  }
  else if (strncmp(tokens[idx], "GUNSG", strlen("GUNSG")) == 0)
  {
    pos->datumid = GUNSG;
  }
  else if (strncmp(tokens[idx], "GEO49", strlen("GEO49")) == 0)
  {
    pos->datumid = GEO49;
  }
  else if (strncmp(tokens[idx], "GRB36", strlen("GRB36")) == 0)
  {
    pos->datumid = GRB36;
  }
  else if (strncmp(tokens[idx], "GUAM", strlen("GUAM")) == 0)
  {
    pos->datumid = GUAM;
  }
  else if (strncmp(tokens[idx], "HAWAII", strlen("HAWAII")) == 0)
  {
    pos->datumid = HAWAII;
  }
  else if (strncmp(tokens[idx], "KAUAI", strlen("KAUAI")) == 0)
  {
    pos->datumid = KAUAI;
  }
  else if (strncmp(tokens[idx], "MAUI", strlen("MAUI")) == 0)
  {
    pos->datumid = MAUI;
  }
  else if (strncmp(tokens[idx], "OAHU", strlen("OAHU")) == 0)
  {
    pos->datumid = OAHU;
  }
  else if (strncmp(tokens[idx], "HERAT", strlen("HERAT")) == 0)
  {
    pos->datumid = HERAT;
  }
  else if (strncmp(tokens[idx], "HJORS", strlen("HJORS")) == 0)
  {
    pos->datumid = HJORS;
  }
  else if (strncmp(tokens[idx], "HONGK", strlen("HONGK")) == 0)
  {
    pos->datumid = HONGK;
  }
  else if (strncmp(tokens[idx], "HUTZU", strlen("HUTZU")) == 0)
  {
    pos->datumid = HUTZU;
  }
  else if (strncmp(tokens[idx], "INDIA", strlen("INDIA")) == 0)
  {
    pos->datumid = INDIA;
  }
  else if (strncmp(tokens[idx], "IRE65", strlen("IRE65")) == 0)
  {
    pos->datumid = IRE65;
  }
  else if (strncmp(tokens[idx], "KERTA", strlen("KERTA")) == 0)
  {
    pos->datumid = KERTA;
  }
  else if (strncmp(tokens[idx], "KANDA", strlen("KANDA")) == 0)
  {
    pos->datumid = KANDA;
  }
  else if (strncmp(tokens[idx], "LIBER", strlen("LIBER")) == 0)
  {
    pos->datumid = LIBER;
  }
  else if (strncmp(tokens[idx], "LUZON", strlen("LUZON")) == 0)
  {
    pos->datumid = LUZON;
  }
  else if (strncmp(tokens[idx], "MINDA", strlen("MINDA")) == 0)
  {
    pos->datumid = MINDA;
  }
  else if (strncmp(tokens[idx], "MERCH", strlen("MERCH")) == 0)
  {
    pos->datumid = MERCH;
  }
  else if (strncmp(tokens[idx], "NAHR", strlen("NAHR")) == 0)
  {
    pos->datumid = NAHR;
  }
  else if (strncmp(tokens[idx], "NAD83", strlen("NAD83")) == 0)
  {
    pos->datumid = NAD83;
  }
  else if (strncmp(tokens[idx], "CANADA", strlen("CANADA")) == 0)
  {
    pos->datumid = CANADA;
  }
  else if (strncmp(tokens[idx], "ALASKA", strlen("ALASKA")) == 0)
  {
    pos->datumid = ALASKA;
  }
  else if (strncmp(tokens[idx], "NAD27", strlen("NAD27")) == 0)
  {
    pos->datumid = NAD27;
  }
  else if (strncmp(tokens[idx], "CARIBB", strlen("CARIBB")) == 0)
  {
    pos->datumid = CARIBB;
  }
  else if (strncmp(tokens[idx], "MEXICO", strlen("MEXICO")) == 0)
  {
    pos->datumid = MEXICO;
  }
  else if (strncmp(tokens[idx], "CAMER", strlen("CAMER")) == 0)
  {
    pos->datumid = CAMER;
  }
  else if (strncmp(tokens[idx], "MINNA", strlen("MINNA")) == 0)
  {
    pos->datumid = MINNA;
  }
  else if (strncmp(tokens[idx], "OMAN", strlen("OMAN")) == 0)
  {
    pos->datumid = OMAN;
  }
  else if (strncmp(tokens[idx], "PUERTO", strlen("PUERTO")) == 0)
  {
    pos->datumid = PUERTO;
  }
  else if (strncmp(tokens[idx], "QORNO", strlen("QORNO")) == 0)
  {
    pos->datumid = QORNO;
  }
  else if (strncmp(tokens[idx], "ROME", strlen("ROME")) == 0)
  {
    pos->datumid = ROME;
  }
  else if (strncmp(tokens[idx], "CHUA", strlen("CHUA")) == 0)
  {
    pos->datumid = CHUA;
  }
  else if (strncmp(tokens[idx], "SAM56", strlen("SAM56")) == 0)
  {
    pos->datumid = SAM56;
  }
  else if (strncmp(tokens[idx], "SAM69", strlen("SAM69")) == 0)
  {
    pos->datumid = SAM69;
  }
  else if (strncmp(tokens[idx], "CAMPO", strlen("CAMPO")) == 0)
  {
    pos->datumid = CAMPO;
  }
  else if (strncmp(tokens[idx], "SACOR", strlen("SACOR")) == 0)
  {
    pos->datumid = SACOR;
  }
  else if (strncmp(tokens[idx], "YACAR", strlen("YACAR")) == 0)
  {
    pos->datumid = YACAR;
  }
  else if (strncmp(tokens[idx], "TANAN", strlen("TANAN")) == 0)
  {
    pos->datumid = TANAN;
  }
  else if (strncmp(tokens[idx], "TIMBA", strlen("TIMBA")) == 0)
  {
    pos->datumid = TIMBA;
  }
  else if (strncmp(tokens[idx], "TOKYO", strlen("TOKYO")) == 0)
  {
    pos->datumid = TOKYO;
  }
  else if (strncmp(tokens[idx], "TRIST", strlen("TRIST")) == 0)
  {
    pos->datumid = TRIST;
  }
  else if (strncmp(tokens[idx], "VITI", strlen("VITI")) == 0)
  {
    pos->datumid = VITI;
  }
  else if (strncmp(tokens[idx], "WAK60", strlen("WAK60")) == 0)
  {
    pos->datumid = WAK60;
  }
  else if (strncmp(tokens[idx], "WGS72", strlen("WGS72")) == 0)
  {
    pos->datumid = WGS72;
  }
  else if (strncmp(tokens[idx], "WGS84", strlen("WGS84")) == 0)
  {
    pos->datumid = WGS84;
  }
  else if (strncmp(tokens[idx], "ZANDE", strlen("ZANDE")) == 0)
  {
    pos->datumid = ZANDE;
  }
  else if (strncmp(tokens[idx], "USER", strlen("USER")) == 0)
  {
    pos->datumid = USER;
  }
  else
  {
    pos->datumid = WGS84;
  }
  idx++;

  // Check latitude standard deviation.
  pos->sd_lat = atof(tokens[idx]);
  idx++;

  // Check longitude standard deviation.
  pos->sd_lon = atof(tokens[idx]);
  idx++;

  // Check height standard deviation.
  pos->sd_hgt = atof(tokens[idx]);
  idx++;

  // Check station ID.
  strncpy(pos->stn_id, tokens[idx], 4);
  idx++;

  // Check differential age.
  pos->diff_age = atof(tokens[idx]);
  idx++;

  // Check solution age.
  pos->sol_age = atof(tokens[idx]);
  idx++;

  // Check number of satellites in view.
  pos->num_sats_in_view = atoi(tokens[idx]);
  idx++;

  // Check number of L1 satellites in use.
  pos->num_sats_L1 = atoi(tokens[idx]);
  idx++;

  // Check number of L1 RTK satellites in use.
  pos->num_sats_L1_RTK = atoi(tokens[idx]);
  idx++;

  // Check number of L2 RTK satellites in use.
  pos->num_sats_L2_RTK = atoi(tokens[idx]);

  return idx;
} /* end oem4_parse_bestpos() */


/*-------------------------------------------------------
 * void oem4_print_status()
 * Prints status information to the screen.
 *-----------------------------------------------------*/
void oem4_print_status(BESTPOS *pos)
{
  printf("OEM4_PRINT_STATUS: status = %d\n", pos->status);
  printf("OEM4_PRINT_STATUS: type = %d\n", pos->type);
  printf("OEM4_PRINT_STATUS: lat = %.12lf\n", pos->lat);
  printf("OEM4_PRINT_STATUS: lon = %.12lf\n", pos->lon);
  printf("OEM4_PRINT_STATUS: hgt = %.12lf\n", pos->hgt);
  printf("OEM4_PRINT_STATUS: undulation = %.12lf\n", pos->undulation);
  printf("OEM4_PRINT_STATUS: datumid = %d\n", pos->datumid);
  printf("OEM4_PRINT_STATUS: sd_lat = %.12lf\n", pos->sd_lat);
  printf("OEM4_PRINT_STATUS: sd_lon = %.12lf\n", pos->sd_lon);
  printf("OEM4_PRINT_STATUS: sd_hgt = %.12lf\n", pos->sd_hgt);
  printf("OEM4_PRINT_STATUS: stn_id = %s\n", pos->stn_id);
  printf("OEM4_PRINT_STATUS: diff_age = %lf\n", pos->diff_age);
  printf("OEM4_PRINT_STATUS: sol_age = %lf\n", pos->sol_age);
  printf("OEM4_PRINT_STATUS: num_sats_in_view = %d\n", pos->num_sats_in_view);
  printf("OEM4_PRINT_STATUS: num_sats_L1 = %d\n", pos->num_sats_L1);
  printf("OEM4_PRINT_STATUS: num_sats_L1_RTK = %d\n", pos->num_sats_L1_RTK);
  printf("OEM4_PRINT_STATUS: num_sats_L2_RTK = %d\n", pos->num_sats_L2_RTK);
  printf("\n");
} /* end oem4_print_status() */


/*-------------------------------------------------------
 * int oem4_write_logfile()
 * Writes status information to a log file.
 *-----------------------------------------------------*/
int oem4_write_logfile(FILE *f_log, BESTPOS *pos)
{
  // Declare variables.
  int bytes = 0;

  // Declare timestamp variables.
  struct timeval ctime;
  struct tm ct;
  char write_time[80] = {0};

  // Get a timestamp and use for log.
  gettimeofday(&ctime, NULL);
  ct = *(localtime ((const time_t*) & ctime.tv_sec));
  strftime(write_time, sizeof(write_time), "20%y%m%d, %H%M%S", &ct);
  snprintf(write_time + strlen(write_time), strlen(write_time), ".%06ld", ctime.tv_usec);

  // Write data to the log file.
  bytes = fprintf(f_log, "%s, %.11lf, %.11lf, %.4lf, %.12lf, %.12lf, %.12lf, %f, %f, %d, %d, %d, %d\n",
                  write_time, pos->lat, pos->lon, pos->hgt, pos->sd_lat, pos->sd_lon, pos->sd_hgt, pos->diff_age,
                  pos->sol_age, pos->num_sats_in_view, pos->num_sats_L1, pos->num_sats_L1_RTK, pos->num_sats_L2_RTK);

  return bytes;
} /* end oem4_write_logfile() */


/*-------------------------------------------------------
 * int oem4_log_bestpos()
 * Send a request to log the BESTPOS message.
 *-----------------------------------------------------*/
int oem4_log_bestpos(int fd, float rate, char *port)
{
  // Declare variables.
  int bytes = 0;
  char OEM4_bestpos[OEM4_STRING_SIZE];
  char *sOEM4_bestpos = &OEM4_bestpos[0];

  // Fill in and send the message.
  snprintf(sOEM4_bestpos, OEM4_STRING_SIZE, "\rfix none\r");
  bytes = serial_send(fd, sOEM4_bestpos, strlen(sOEM4_bestpos));

  snprintf(sOEM4_bestpos, OEM4_STRING_SIZE, "\rlog %s bestpos ontime %f\r", port, rate);
  bytes = serial_send(fd, sOEM4_bestpos, strlen(sOEM4_bestpos));

  return bytes;
} /* end oem4_log_bestpos() */


/*-------------------------------------------------------
 * int oem4_unlogall()
 * Send a request to unlogall GPS messages.
 *-----------------------------------------------------*/
int oem4_unlogall(int fd)
{
  // Declare variables.
  int bytes = 0;
  char OEM4_unlogall[OEM4_STRING_SIZE];
  char *sOEM4_unlogall = &OEM4_unlogall[0];
  snprintf(sOEM4_unlogall, OEM4_STRING_SIZE, "\runlogall\r");

  // Send the message.
  bytes = serial_send(fd, sOEM4_unlogall, strlen(sOEM4_unlogall));

  return bytes;
} /* end oem4_unlogall() */


/*-------------------------------------------------------
 * int oem4_set_baud()
 * Sets the baud rate for the given port.
 *-----------------------------------------------------*/
int oem4_set_baud(int fd, int baud, char *port)
{
  // Declare variables.
  int bytes = 0;
  char *parity = (char *)"N";
  int databits = 8;
  int stopbits = 1;
  char *handshake = (char *)"N";
  char *echo = (char *)"OFF";
  char *dbreak = (char *)"ON";
  char OEM4_baud[OEM4_STRING_SIZE];
  char *sOEM4_baud = &OEM4_baud[0];
  snprintf(sOEM4_baud, OEM4_STRING_SIZE, "\rcom %s,%d,%s,%d,%d,%s,%s,%s\r", port, baud, parity, databits, stopbits, handshake, echo, dbreak);

  // Send the message.
  bytes = serial_send(fd, sOEM4_baud, strlen(sOEM4_baud));

  return bytes;
} /* end oem4_log_baud() */


/*-------------------------------------------------------
 * int oem4_posave()
 * Enable or disable position averaging for the base station.
 *-----------------------------------------------------*/
int oem4_posave(int fd, int state, float maxtime, float maxhstd, float maxvstd)
{
  // Declare variables.
  int bytes = 0;
  char *s_state;
  char OEM4_posave[OEM4_STRING_SIZE];
  char *sOEM4_posave = &OEM4_posave[0];

  if (state)
  {
    s_state = (char *)"ON";
    snprintf(sOEM4_posave, OEM4_STRING_SIZE, "\rposave %s %f %f %f\r", s_state, maxtime, maxhstd, maxvstd);
  }
  else
  {
    s_state = (char *)"OFF";
    snprintf(sOEM4_posave, OEM4_STRING_SIZE, "\rposave %s\r", s_state);
  }

  // Send the message.
  bytes = serial_send(fd, sOEM4_posave, strlen(sOEM4_posave));

  return bytes;
} /* end oem4_posave() */
