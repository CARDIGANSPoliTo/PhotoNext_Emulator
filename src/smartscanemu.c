/*******************************************************************************
* included libraries
*******************************************************************************/
#include "../include/smartscanemu.h"

/*******************************************************************************
* global variables
*******************************************************************************/
volatile sig_atomic_t stop_process;

pthread_mutex_t *lock_m;

// ssi variables
uint8_t ssi_state;

unsigned int raw_speed;
unsigned int cont_speed;
unsigned int scan_time_us;

uint32_t scan_frame_count;
uint32_t cont_frame_count;

int s_socket; // shared send socket (needs mutex on write);
struct sockaddr_in s_sin;

SSI_CONFIG board_config;

/*******************************************************************************
* signal handling
*******************************************************************************/
void sigint_handler(int signal) {
  printf("Exiting emulator.\n");
  stop_process = 1;
  exit(0);
}

/*******************************************************************************
* custom functions
*******************************************************************************/
void board_init()
{
  board_config.ssi_demo = 0;
  board_config.ssi_gratings = 16;
  board_config.ssi_channels = 4;

  board_config.ssi_raw_speed = 0;
  board_config.ssi_cont_speed = 25;
  board_config.ssi_scan_speed = 400;
  
  board_config.ssi_first_fr = 0;

  strncpy(board_config.ssi_netif, "eth0", IF_NAMESIZE);
  strncpy(board_config.ssi_smsc_ip, "10.0.0.150", 16);
  strncpy(board_config.ssi_host_ip, "10.0.0.2", 16);
  strncpy(board_config.ssi_subnet, "255.255.255.0", 16);
  strncpy(board_config.ssi_gateway, "10.0.0.2", 16);

  board_config.ssi_serial      = 123456;
  board_config.ssi_log_level   = 7;

  ssi_state = SSI_STATE_STAND_BY;

  raw_speed = 0;
  cont_speed = 0;
  scan_time_us = 0;

  scan_frame_count = 0;
  cont_frame_count = 0;

  printf("SSI board initalised.\n");

  return;
};

void update_cont_tx_speed(SSI_CONFIG *conf)
{
  cont_speed = conf->ssi_cont_speed*scan_time_us;
  printf("Set continuous data speed tx to %d us.\n", cont_speed);

  return;
};

void update_raw_tx_speed(SSI_CONFIG *conf)
{
  raw_speed = conf->ssi_raw_speed;
  printf("Set scan data speed tx to %d Hertz.\n", raw_speed);

  return;
};

void update_scan_time_us(SSI_CONFIG *conf)
{
  scan_time_us = conf->ssi_scan_speed;
  printf("Set scan time to %d us.\n", scan_time_us);

  return;
};

// decode from message protocol
uint16_t decode_scan_time_us(uint16_t scancode)
{
  uint16_t tmp_data = scancode;
  uint16_t steps = 0;
  uint8_t cycle_step_us = 0;

  uint8_t step_code = 0;
  uint8_t cycle_code = 0;

  if((tmp_data & 0x8000) == 0) // mode bit set to 0
  {
    step_code = (tmp_data & 0x0007);
    switch(step_code)
    {
      case 0:
        steps = 400;
        break;
      case 1:
        steps = 200;
        break;
      case 2:
        steps = 100;
        break;
      case 3:
        steps = 50;
        break;
      default:
        steps = 400;
        break;
    }

    cycle_code = (tmp_data & 0x0038) >> 3;
    switch (cycle_code) {
      case 0:
        cycle_step_us = 1;
        break;
      case 1:
        cycle_step_us = 2;
        break;
      case 2:
        cycle_step_us = 5;
        break;
      case 3:
        cycle_step_us = 10;
        break;
      case 4:
        cycle_step_us = 20;
        break;
      case 5:
        cycle_step_us = 50;
        break;
      default:
        cycle_step_us = 1;
        break;
    }
  }
  else
  {
    steps = (tmp_data & 0x01ff);

    cycle_code = (tmp_data & 0x1c00) >> 10;
    switch (cycle_code) {
      case 0:
        cycle_step_us = 1;
        break;
      case 1:
        cycle_step_us = 2;
        break;
      case 2:
        cycle_step_us = 5;
        break;
      case 3:
        cycle_step_us = 10;
        break;
      case 4:
        cycle_step_us = 20;
        break;
      case 5:
        cycle_step_us = 50;
        break;
      default:
        cycle_step_us = 1;
        break;
    }
  }

  return steps*cycle_step_us;
}

uint8_t decode_gratings(uint16_t chanformat)
{
  return ((chanformat & 0x01F0) >> 4);
}

uint8_t decode_channels(uint16_t chanformat)
{
  return (chanformat & 0x000F);
}


// encoding for message protocol
uint16_t encode_scan_time_us(uint16_t ssi_scan_speed_us)
{
  // TODO add proper encoding
  return 0x001a;
}

uint16_t encode_chanformat(uint8_t channels, uint8_t gratings)
{
  uint16_t chanformat = 0x4000;
  // cast to 16 bits to match chanformat length
  uint16_t ch16 = (uint16_t) channels;
  uint16_t gr16 = (uint16_t) gratings;

  chanformat |= (ch16 << 4);
  chanformat |= gr16;

  return chanformat;
}

int parse_maintenance(uint8_t* buffer, size_t len, SSI_CONFIG *conf)
{
  int error_code = STATUS_OK;

  size_t current_index = 0;
  int i;

  uint8_t cmd, cmd_len, *cmd_data;
  uint8_t upd_scan_speed = 0, upd_cont_speed = 0, upd_scan_time = 0;

  HD_MAINTENANCE header;

  printf("Parse maintenance message.\n");

  if(!buffer) // check buffer pointer
  {
    printf("Buffer pointer is NULL.\n");
    error_code = STATUS_ERROR;
  }

  if(!error_code && (len < HD_MAINTENANCE_SIZE || len > MSG_MAINTENANCE_SIZE)) // check buffer length
  {
    printf("Buffer length is invalid.\n");
    error_code = STATUS_ERROR;
  }

  if(!error_code && (((len - HD_MAINTENANCE_SIZE) % 4) != 0))  // maintenance payload is 32 bits aligned
  {
    printf("Payload is not correctly aligned.\n");
    error_code = STATUS_ERROR;
  }

  if(!error_code) // start message parse
  {
    current_index += read_32((void *)(buffer + current_index), &(header.ulCodeStamp), BE);
    current_index += read_8((void *)(buffer + current_index), &(header.ucSpare));
    current_index += read_8((void *)(buffer + current_index), &(header.ucState));

    upd_scan_speed = 0;
    upd_cont_speed = 0;
    upd_scan_time = 0;

    // fields to be parsed before updating the configuration structure
    uint16_t scancode;
    uint16_t chanformat;

    while(current_index < len && !error_code)
    {
      // parse command
      current_index += read_8((void *) (buffer + current_index), &cmd);
      current_index += read_8((void *) (buffer + current_index), &cmd_len);

      cmd_data = (uint8_t *) malloc(cmd_len * sizeof(uint8_t));

      if(cmd_data)
      {
        for(i=0; i<cmd_len; i++)
        {
          current_index += read_8((void *) (buffer + current_index), cmd_data + i);
        }
        switch(cmd)
        {
          case CMD_SET_STATE_CMD:
            read_8(cmd_data, &(ssi_state));
            break;
          case CMD_SET_DEMO_MODE_CMD:
            read_8(cmd_data, &(conf->ssi_demo));
            break;
          case CMD_SET_SCAN_RATE_CMD:
            read_16(cmd_data, &(conf->ssi_raw_speed), BE);
            upd_scan_speed = 1;
            break;
          case CMD_SET_CONT_RATE_CMD:
            read_16(cmd_data, &(conf->ssi_cont_speed), BE);
            upd_cont_speed = 1;
            break;
          case CMD_SET_CH_FORMAT_CMD:
            read_16(cmd_data, &(chanformat), BE);
            conf->ssi_channels = decode_channels(chanformat);
            conf->ssi_gratings = decode_gratings(chanformat);
            break;
          // case CMD_SET_CH_THRESH_CMD:
          //   read_16(cmd_data, &(conf->ssi_threshold), BE);
          //   break;
          // case CMD_RET_DBG_VAL_CMD:
          //   read_16(cmd_data, &(conf->ssi_debug), BE);
          //   break;
          case CMD_SET_SCAN_BEG_CMD:
            read_16(cmd_data, &(conf->ssi_first_fr), BE);
            break;
          case CMD_SET_SCAN_SP_CMD:
            read_16(cmd_data, &(scancode), BE);
            conf->ssi_scan_speed = decode_scan_time_us(scancode);
            upd_scan_time = 1;
            break;
          // case CMD_RET_SCAN_DIR_CMD:
          //   read_16(cmd_data, &(conf->ssi_scandir), BE);
          //   break;
          // case CMD_RET_SCAN_CNT_CMD:
          //   read_16(cmd_data, &(conf->ssi_scancnt), BE);
          //   break;
          // case CMD_RET_SW_VER_CMD:
          //   read_16(cmd_data, &(conf->ssi_version), BE);
          //   break;
          // case CMD_SET_IP_ADDR_CMD:
          //   for(i=0; i<cmd_len; i++)
          //   {
          //     conf->ssi_ip_addr[i] = (uint8_t) *(cmd_data + i);
          //   }
          //   break;
          // case CMD_SET_SUBNET_CMD:
          //   for(i=0; i<cmd_len; i++)
          //   {
          //     conf->ssi_subnet[i] = (uint8_t) *(cmd_data + i);
          //   }
          //   break;
          // case CMD_RET_MAC_ADD_CMD:
          //   for(i=0; i<cmd_len; i++)
          //   {
          //     conf->ssi_mac_addr[i] = (uint8_t) *(cmd_data + i);
          //   }
          //   break;
          // case CMD_SET_GATEWAY_CMD:
          //   for(i=0; i<cmd_len; i++)
          //   {
          //     conf->ssi_gateway[i] = (uint8_t) *(cmd_data + i);
          //   }
          //   break;
          // case CMD_SET_CPU_UTC_CMD:
          //   read_32(cmd_data, &(conf->ssi_utc_local), BE);
          //   break;
          // case CMD_RET_SERIAL_CMD:
          //   read_32(cmd_data, &(conf->ssi_serial), BE);
          //   break;
          default:
            printf("Command not recognised: %u.\n", cmd);
            break;
        }
        free(cmd_data);
      }
      else
      {
        printf("Unable to allocate memory.\n");
        error_code = STATUS_ERROR;
      }
    }

    if(upd_scan_time)
    {
      update_scan_time_us(conf);
    }

    if(upd_cont_speed)
    {
      update_cont_tx_speed(conf);
    }

    if(upd_scan_speed)
    {
      update_raw_tx_speed(conf);
    }

    ssi_dump_config(conf);
  }
  return error_code;
};

size_t create_maintenance(uint8_t *message, SSI_CONFIG *conf)
{
  uint8_t cmd = 129;
  uint8_t cmd_len = 1;
  size_t current_index = 0;

  uint16_t tmp16_val = 0;

  int i;

  memset((void *) message, 0, MSG_LIMIT_MTU);
  current_index = 0;

  current_index += ssi_write_maint_header(message + current_index);

  cmd = CMD_RET_STATE_CMD;
  cmd_len = CMD_RET_STATE_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  current_index += write_8(&(ssi_state), message + current_index);

  cmd = CMD_RET_DEMO_MODE_CMD;
  cmd_len = CMD_RET_DEMO_MODE_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  current_index += write_8(&(conf->ssi_demo), message + current_index);

  cmd = CMD_RET_SCAN_TX_CMD;
  cmd_len = CMD_RET_SCAN_TX_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  current_index += write_16(&(conf->ssi_raw_speed), message + current_index, BE);

  cmd = CMD_RET_DATA_CODE_CMD;
  cmd_len = CMD_RET_DATA_CODE_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  current_index += write_16(&(conf->ssi_cont_speed), message + current_index, BE);

  // cmd = CMD_RET_CH_FORMAT_CMD;
  // cmd_len = CMD_RET_CH_FORMAT_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // current_index += write_16(&(conf->ssi_chanformat), message + current_index, BE);

  // cmd = CMD_RET_SCAN_FREQ_CMD;
  // cmd_len = CMD_RET_SCAN_FREQ_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // current_index += write_16(&(conf->ssi_scanbeg), message + current_index, BE);

  cmd = CMD_RET_SCAN_CODE_CMD;
  cmd_len = CMD_RET_SCAN_CODE_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  tmp16_val = encode_scan_time_us(conf->ssi_scan_speed);
  current_index += write_16(&(tmp16_val), message + current_index, BE);

  // cmd = CMD_RET_SW_VER_CMD;
  // cmd_len = CMD_RET_SW_VER_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // current_index += write_16(&(conf->ssi_version), message + current_index, BE);

  // cmd = CMD_RET_IP_ADDR_CMD;
  // cmd_len = CMD_RET_IP_ADDR_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // for(i=0; i<4; i++)
  // {
  //   current_index += write_8((uint8_t *)&(conf->ssi_ip_addr[i]), message + current_index);
  // }

  // cmd = CMD_RET_SUBNET_CMD;
  // cmd_len = CMD_RET_SUBNET_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // for(i=0; i<4; i++)
  // {
  //   current_index += write_8((uint8_t *)&(conf->ssi_subnet[i]), message + current_index);
  // }

  // cmd = CMD_RET_MAC_ADD_CMD;
  // cmd_len = CMD_RET_MAC_ADD_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // for(i=0; i<6; i++)
  // {
  //   current_index += write_8((uint8_t *)&(conf->ssi_mac_addr[i]), message + current_index);
  // }

  // cmd = CMD_RET_GATEWAY_CMD;
  // cmd_len = CMD_RET_GATEWAY_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // for(i=0; i<4; i++)
  // {
  //   current_index += write_8((uint8_t *)&(conf->ssi_gateway[i]), message + current_index);
  // }

  // cmd = CMD_RET_UTC_CMD;
  // cmd_len = CMD_RET_UTC_LEN;
  // current_index += write_8(&(cmd), message + current_index);
  // current_index += write_8(&(cmd_len), message + current_index);
  // current_index += write_32(&(conf->ssi_utc_local), message + current_index, BE);

  cmd = CMD_RET_SERIAL_CMD;
  cmd_len = CMD_RET_SERIAL_LEN;
  current_index += write_8(&(cmd), message + current_index);
  current_index += write_8(&(cmd_len), message + current_index);
  current_index += write_32(&(conf->ssi_serial), message + current_index, BE);

  current_index += ssi_write_maint_padding(message + current_index, current_index);

  return current_index;
};

size_t create_scan(uint8_t *message, size_t len)
{
  size_t current_index = 0;

  uint8_t  tmp8 = 0;
  uint16_t tmp16 = 0;
  uint32_t tmp32 = 0;

  struct timespec current_time;

  int i = 0;

  printf("Create scan message.\n");

  if(!message)
  {
    printf("Message pointer is NULL.\n");
  }
  else
  {
    memset((void *) message, 0, len);

    tmp16 = 834;  // usFrameSize
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp8 = 9; // ucHdrSizex4
    current_index += write_8(&tmp8, message + current_index);
    tmp8 = 255; // ucFrameFormat
    current_index += write_8(&tmp8, message + current_index);
    tmp32 = scan_frame_count++; // ulFrameCount
    current_index += write_32(&tmp32, message + current_index, BE);
    clock_gettime(CLOCK_REALTIME, &current_time);
    tmp32 = (uint32_t) current_time.tv_sec; // ulTimeStampH
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = (uint32_t) current_time.tv_nsec/1000; // ulTimeStampL
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = (uint32_t) current_time.tv_sec; // ulTimeCodeH
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp16 = 400; // usTimeInterval (usecs)
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 400; // usNrSteps
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 0; // usMinChannel;
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 399; // usMaxChannel;
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp32 = 0; // ulMinWaveFreq
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = 0; // ulMaxWaveFreq
    current_index += write_32(&tmp32, message + current_index, BE);

    for(i=0; i<400; i++)
    {
      tmp16 = rand()%51199; // data;
      current_index += write_16(&tmp16, message + current_index, BE);
    }
  }

  return current_index;
};

size_t create_cont(uint8_t *message, size_t len, SSI_CONFIG *conf)
{
  size_t current_index = 0;

  uint8_t  tmp8 = 0;
  uint16_t tmp16 = 0;
  uint32_t tmp32 = 0;

  int i, payload_size = 0, frames = 0, channels = 4, gratings = 16;

  printf("Create continuous message.\n");

  if(!message)
  {
    printf("Message pointer is NULL.\n");
  }
  else
  {
    memset((void *) message, 0, len);

    channels = conf->ssi_channels;
    gratings = conf->ssi_gratings;

    frames = (MSG_LIMIT_MTU - HD_CONT_DATA_SIZE)/(gratings*channels*sizeof(uint16_t));
    payload_size = frames * (gratings * channels * sizeof(uint16_t));

    tmp16 = HD_CONT_DATA_SIZE + payload_size - 2;  // usFrameSize
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp8 = 9; // ucHdrSizex4
    current_index += write_8(&tmp8, message + current_index);
    tmp8 = 0x00 | ((gratings == 16 ? 0 : gratings) << 4) | channels; // ucFrameFormat
    current_index += write_8(&tmp8, message + current_index);
    tmp32 = cont_frame_count++; // ulFrameCount
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = (uint32_t) time(NULL); // ulTimeStampH
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = rand(); // ulTimeStampL
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = (uint32_t) time(NULL); // ulTimeCodeH
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp16 = 400; // usTimeInterval (usecs)
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 0; // usSpare
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 0; // usMinChannel;
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp16 = 399; // usMaxChannel;
    current_index += write_16(&tmp16, message + current_index, BE);
    tmp32 = 0; // ulMinWaveFreq
    current_index += write_32(&tmp32, message + current_index, BE);
    tmp32 = 0; // ulSpare
    current_index += write_32(&tmp32, message + current_index, BE);

    for(i=0; i<payload_size; i+=(sizeof(uint16_t)))
    {
      // tmp16 = (rand()%400) * LASER_CHANNEL_MULT; // data;
      tmp16 = (183 + (rand()%2 == 1 ? 1 : -1)*rand()%50) * LASER_CHANNEL_MULT; // data;
      current_index += write_16(&tmp16, message + current_index, BE);
    }
  }

  return current_index;
};

void *scan_th(void *args)
{
  uint8_t message[MSG_LIMIT_MTU] = {0};
  size_t msg_len = 0;

  struct sockaddr_in dest;

  dest.sin_family = AF_INET;
  dest.sin_port = htons(PORT_RX_SCAN);

  if(inet_aton(SERVER_IP_ADD, &(dest.sin_addr)) == 0)
  {
    printf("Invalid destination IP address.\n");
    exit(1);
  }

  while(!stop_process)
  {
    if(raw_speed != 0)
    {
      if((msg_len = create_scan(message, MSG_LIMIT_MTU)) > 0)
      {
        pthread_mutex_lock(lock_m);
        if((sendto(s_socket, message, msg_len, 0, (struct sockaddr *) &dest, (socklen_t) sizeof(dest))) == -1)
        {
          printf("Unable to send message.\n");
        }
        else
        {
          printf("Sent packet of length %ld from %s:%d to %s:%d.\n", sizeof(buffer_scan), inet_ntoa(s_sin.sin_addr), ntohs(s_sin.sin_port), inet_ntoa(dest.sin_addr), ntohs(dest.sin_port));
        }
        pthread_mutex_unlock(lock_m);

      }
      usleep(1000000 / raw_speed);
    }
    else
    {
      sleep(1);
    }
  }
  return (void *)0;
};

void *cont_th(void *args)
{
  uint8_t message[MSG_LIMIT_MTU] = {0};
  size_t msg_len = 0;

  struct sockaddr_in dest;

  dest.sin_family = AF_INET;
  dest.sin_port = htons(PORT_RX_CONT);

  if(inet_aton(SERVER_IP_ADD, &(dest.sin_addr)) == 0)
  {
    printf("Invalid destination IP address.\n");
    exit(1);
  }

  while(!stop_process)
  {
    if(cont_speed != 0)
    {
      if((msg_len = create_cont(message, MSG_LIMIT_MTU, &board_config)) > 0)
      {
        pthread_mutex_lock(lock_m);
        if((sendto(s_socket, message, msg_len, 0, (struct sockaddr *) &dest, (socklen_t) sizeof(dest))) == -1)
        {
          printf("Unable to send message.\n");
        }
        else
        {
          printf("Sent packet of length %ld from %s:%d to %s:%d.\n", msg_len, inet_ntoa(s_sin.sin_addr), ntohs(s_sin.sin_port), inet_ntoa(dest.sin_addr), ntohs(dest.sin_port));
        }
        pthread_mutex_unlock(lock_m);
      }
      usleep(cont_speed);
    }
    else
    {
      sleep(1);
    }
  }

  printf("End sequence.\n");
  return (void *)0;
};

/*******************************************************************************
* main program
*******************************************************************************/
int main (int argc, char **argv){

  signal(SIGINT, sigint_handler);

  pthread_t c_tid = -1, s_tid = -1;
  void *result; // thread exit result

  int rec_diag_msg_cnt = 0;

  int d_socket, m_socket;
  struct sockaddr_in d_sin, m_sin, src, dest;

  socklen_t l = sizeof(src);

  uint8_t rx_buffer[MSG_LIMIT_MTU];
  uint8_t tx_buffer[MSG_LIMIT_MTU];
  int rec_len;

  size_t msg_len = 0;

  struct timeval select_to;
  select_to.tv_sec = 20;
  select_to.tv_usec = 0;

  fd_set read_fds;
  int fd_ready;

  stop_process = 0;

  // init mutex for socket critical section
  lock_m = (pthread_mutex_t*)malloc(sizeof(pthread_mutex_t));
  if (!lock_m) {
    printf("Unable to allocate lock mutex.\n");
    exit(1);
  }

  pthread_mutex_init(lock_m, NULL);

  board_init();

  srand(time(NULL));

  printf("Emulator started.\n");

  if((d_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf("Unable to open diagnostic socket.\n");
    exit(1);
  }

  if((m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == - 1)
  {
    printf("Unable to open maintenance socket.\n");
    exit(1);
  }

  if((s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
  {
    printf("Unable to open scan data socket.\n");
    exit(1);
  }

  d_sin.sin_family = AF_INET;
  m_sin.sin_family = AF_INET;
  s_sin.sin_family = AF_INET;
  dest.sin_family = AF_INET;

#if EMU_LOCAL == 1
  d_sin.sin_port = htons(30011);
  m_sin.sin_port = htons(30012);
#else
  d_sin.sin_port = htons(PORT_RX_DIAG);
  m_sin.sin_port = htons(PORT_RX_MAIN);
#endif
  s_sin.sin_port = htons(PORT_TX_CLIENT);
  dest.sin_port = htons(PORT_RX_DIAG);
  //
  if(inet_aton(LISTEN_IP_ADD, &(d_sin.sin_addr)) == 0)
  {
    printf("Invalid listen IP address for diagnostic.\n");
    exit(1);
  }
  if(inet_aton(LISTEN_IP_ADD, &(m_sin.sin_addr)) == 0)
  {
    printf("Invalid listen IP address for maintenance.\n");
    exit(1);
  }
  if(inet_aton(CLIENT_IP_ADD, &(s_sin.sin_addr)) == 0)
  {
    printf("Invalid source IP address.\n");
    exit(1);
  }
  if(inet_aton(SERVER_IP_ADD, &(dest.sin_addr)) == 0)
  {
    printf("Invalid destination IP address.\n");
    exit(1);
  }

  if(bind(d_socket, (struct sockaddr*)&(d_sin), sizeof(d_sin)) == -1)
  {
    printf("Diagnostic socket bind failed.\n");
    exit(1);
  }
  if(bind(m_socket, (struct sockaddr*)&(m_sin), sizeof(m_sin)) == -1)
  {
    printf("Maintenance socket bind failed.\n");
    exit(1);
  }
  if(bind(s_socket, (struct sockaddr*)&(s_sin), sizeof(s_sin)) == -1)
  {
    printf("Send socket bind failed.\n");
    exit(1);
  }

  printf("Open diagnostic socket on %s:%d.\n", inet_ntoa(d_sin.sin_addr), ntohs(d_sin.sin_port));
  printf("Open maintenance socket on %s:%d.\n", inet_ntoa(m_sin.sin_addr), ntohs(m_sin.sin_port));
  printf("Open send socket on %s:%d.\n", inet_ntoa(s_sin.sin_addr), ntohs(s_sin.sin_port));

  pthread_create(&s_tid, NULL, scan_th, NULL);
  pthread_create(&c_tid, NULL, cont_th, NULL);

  while(!stop_process)
  {
    FD_ZERO(&read_fds);
    FD_SET(d_socket, &read_fds);
    FD_SET(m_socket, &read_fds);

    fd_ready = select(FD_SETSIZE, &read_fds, 0, 0, &select_to);

    if(fd_ready < 0)
    {
      printf("Select failed.\n");
      exit(1);
    }

    if(FD_ISSET(d_socket, &read_fds)) { // diagnostic message socket
      if((rec_len = recvfrom(d_socket, rx_buffer, MSG_LIMIT_MTU, 0, (struct sockaddr *) &(src), &l)) <0 )
      {
        printf("Unable to read message.\n");
      }
      else
      {
        rec_diag_msg_cnt++;

        printf("Received packet of size %d from %s:%d on %s:%d.\n", rec_len, inet_ntoa(src.sin_addr), ntohs(src.sin_port), inet_ntoa(d_sin.sin_addr), ntohs(d_sin.sin_port));

        if(rec_diag_msg_cnt == 1) // set operational state after 1 diagnostic message
        {
          ssi_state = SSI_STATE_OPERATIONAL;
        }

        if((ssi_create_diagnostic_msg(tx_buffer, MSG_DIAGNOSTIC_SIZE, ssi_state)) == STATUS_OK)
        {
          dest.sin_port = htons(PORT_RX_DIAG);
          pthread_mutex_lock(lock_m);
          if((sendto(s_socket, tx_buffer, MSG_DIAGNOSTIC_SIZE, 0, (struct sockaddr *) &dest, (socklen_t) sizeof(dest))) == -1)
          {
            printf("Unable to send message.\n");
          }
          else
          {
            printf("Sent packet of length %ld from %s:%d to %s:%d.\n", (size_t) MSG_DIAGNOSTIC_SIZE, inet_ntoa(s_sin.sin_addr), ntohs(s_sin.sin_port), inet_ntoa(dest.sin_addr), ntohs(dest.sin_port));
          }
          pthread_mutex_unlock(lock_m);
        }
      }
      FD_CLR(d_socket, &read_fds);
    }
    if(FD_ISSET(m_socket, &read_fds)) { // maintenance message socket
      if((rec_len = recvfrom(m_socket, rx_buffer, MSG_LIMIT_MTU, 0, (struct sockaddr *) &(src), &l)) <0 )
      {
        printf("Unable to read message.\n");
      }
      else
      {
        printf("Received packet of size %d from %s:%d on %s:%d.\n", rec_len, inet_ntoa(src.sin_addr), ntohs(src.sin_port), inet_ntoa(m_sin.sin_addr), ntohs(m_sin.sin_port));

        parse_maintenance(rx_buffer, rec_len, &board_config);

        msg_len = create_maintenance(tx_buffer, &board_config);

        dest.sin_port = htons(PORT_RX_MAIN);
        pthread_mutex_lock(lock_m);
        if((sendto(s_socket, tx_buffer, msg_len, 0, (struct sockaddr *) &dest, (socklen_t) sizeof(dest))) == -1)
        {
          printf("Unable to send message.\n");
        }
        else
        {
          printf("Sent packet of length %ld from %s:%d to %s:%d.\n", (size_t) MSG_DIAGNOSTIC_SIZE, inet_ntoa(s_sin.sin_addr), ntohs(s_sin.sin_port), inet_ntoa(dest.sin_addr), ntohs(dest.sin_port));
        }
        pthread_mutex_unlock(lock_m);
      }
      FD_CLR(m_socket, &read_fds);
    }
  }

  close(d_socket);
  printf("Closing diagnostic socket.\n");
  close(m_socket);
  printf("Closing maintenance socket.\n");
  close(s_socket);
  printf("Closing send socket.\n");

  pthread_join(c_tid, &result);
  pthread_join(s_tid, &result);

  pthread_mutex_destroy(lock_m);
  free(lock_m);

  return 0;
}
