#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "dirent.h"

#define SIMCONFIG "/data/simconfig"
#define PORTNAME "/dev/smd8"

int NETLIGHT=36,
    SIM_DET=77,
    TOGGLE=79,
    SIM_SEL=1020;

int gpioRead(int gpio)
{
  char buf[64];
  int fd, ret;
    sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
    if ((fd = open(buf, O_RDONLY)) < 0)
    return -1;
  if ((ret = read(fd, buf, 1)) > 0)
  {
    if (*buf == '0') ret=0;
    else
     if (*buf == '1') ret=1;
  }
  close(fd);
  return ret;
}

int gpioSet(int gpio, int value)
{
  char buf[64];
  int fd, ret;
    sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);    
  if ((fd = open(buf, O_WRONLY)) < 0)
    return -1;
    sprintf(buf, "%d", value);
    ret = write(fd, buf, 1);
  close(fd);
  return ret;
}

int gpioSetDirection(int gpio, char *direction)
{
  char buf[64];
  int fd, ret;
    sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);    
  if ((fd = open(buf, O_WRONLY)) < 0)
    return -1;
  sprintf(buf, "%s", direction);
  ret = write(fd, buf, 3);
  close(fd);
  return ret;
}

int simSw(int port)
{
  printf("Switching to port %d\n",port);
  gpioSet(SIM_DET,0);
  usleep(30000);
  gpioSet(SIM_SEL,port);
  usleep(30000);
  gpioSet(SIM_DET,1); 
}

int writeToFile(char *filename, int value)
{
  char command[128];
  sprintf(command, "echo %d > %s", value, filename);
  int ret=system(command);
  return ret;
}

char readFromFile(char *filename, char *pbuf)
{
  char ret=1;
  if (pbuf == NULL)
    return ret;

  int string_size, read_size;
  FILE *handler = fopen(filename, "r");
  if (handler)
  {
    pbuf[1] = '\0';
    if (fread(pbuf, sizeof(char), 1, handler))
      ret=0;
    fclose(handler);
  }
  return ret;
}

// from https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}

int main(int argc, char **argv)
{
  char old_port[2];
  int n,
    i=0,
    new_port=0,
    new_val,
    last_val=0,
    count=0,
    stat;
  readFromFile(SIMCONFIG, old_port);

  printf("START!\n");

  printf("Port from file %s\n",old_port);
  if (old_port[0]=='0')
  {
    printf("IF port %s\n",old_port);
    new_port=1;
  }  
  else
  {
    printf("ELSE port %s\n",old_port);
  }
  simSw(new_port);
//######################
  int fd=open(PORTNAME,O_RDWR|O_NOCTTY|O_SYNC);

  if (fd < 0)
  {
    printf ("error %d opening %s: %s", errno, PORTNAME, strerror (errno));
    return;
  }

  set_interface_attribs (fd, B1152000, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking
  while (i < 50)
  {
    write(fd, "\r\nAT+CPIN?\r\n", 12);           // send 7 character greeting
    usleep((12 + 25)*200);            // sleep enough to transmit the 7 plus
                                     // receive 25:  approx 100 uS per char transmit
    char buf[100];
    n=read(fd,buf,sizeof buf);
    printf("READ from tty: %s\n", buf);

    if(strstr(buf, "OK"))
    {
      printf("FINDED!!!\n");
      break;
    }

    usleep(100000);
    i++;
  }

  if(i>=50)
  {
    if(new_port==1)
      new_port=0;
    else
      new_port=1;
    simSw(new_port);
  }
  writeToFile(SIMCONFIG,new_port);
  printf("I::%d\n", i);

  gpioSetDirection(NETLIGHT,"in");
  
  count=7;
  i=1;
  new_val=gpioRead(NETLIGHT);
  last_val=new_val;
  stat=0;
  while(1)
  {
    new_val=gpioRead(NETLIGHT);
    i++;
    if(new_val==last_val){
      count++;
    }
    else{
      if(count>7)
      {
        stat=1;
      }
      else
      {
        if(count<8 && count>1)
        {
          stat=1;
          i=0;
          gpioRead(TOGGLE) ? gpioSet(TOGGLE,0) : gpioSet(TOGGLE,1);
        }
        else
        {
          stat=2;
        }
      }
      count=0;
    }

    if(stat==0)
    {
      if(i>1)
      {
        gpioRead(TOGGLE) ? gpioSet(TOGGLE,0) : gpioSet(TOGGLE,1);
        i=0;
      }
    }

    if(stat==1)
    {
      if(i>7)
      {
        gpioRead(TOGGLE) ? gpioSet(TOGGLE,0) : gpioSet(TOGGLE,1);
        i=0;
      }
    }

    if(stat==2)
    {
      gpioSet(TOGGLE,1);
      i=0;
    }
    printf("NEW: %d; TOGGLE: %d; STAT: %d; COUNT: %d; I: %d\n",new_val,gpioRead(TOGGLE),stat,count,i);
    usleep(100000);
    last_val=new_val;
  }

  printf("DONE!\n");
}