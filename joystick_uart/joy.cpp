/*
Ordoid XU4 UART Communication
*/

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>     // c standard general utilities
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>     // c++ input / output stream objects

// Joystick
#include <sys/ioctl.h>
#include <linux/joystick.h>

// Timer
#include <cstdio>
#include <ctime>

#define UART_DEVICE_USB0 "/dev/ttyUSB0" //Either /dev/ttyUSB0 or /dev/ttySAC0 depending on XBee connection method
#define UART_DEVICE_USB1 "/dev/ttyUSB1"
#define UART_DEVICE_SAC "/dev/ttySAC0"

#define JOY_DEV "/dev/input/js0" //
#define CONTROL_RANGE 1000
#define DATA_SPEED 0.011458334 // = 1/(960/11) 960 bytes per sec at 9600 baud, 1 set of data is 11 bytes,

/* Open File Descriptor */

int UART0_Send(int fd, char *send_buf, int data_len)
{
	int len = 0;
	len = write(fd, send_buf, data_len);
	if(len == data_len)
	{
		return(len);
	}
	else
	{
		tcflush(fd,TCOFLUSH);
		return(0);
	}
}


int main (void) {

  //============== TIMER ===============

  std::clock_t start;
  double duration;


  //============== JOYSTICK ======================

  int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, x;
  char *button=NULL, name_of_joystick[80];
  struct js_event js;

  if( ( joy_fd = open( JOY_DEV , O_RDONLY)) == -1 ) {
    printf( "Couldn't open joystick\n" );
    return -1;
  }

  ioctl( joy_fd, JSIOCGAXES, &num_of_axis );
  ioctl( joy_fd, JSIOCGBUTTONS, &num_of_buttons );
  ioctl( joy_fd, JSIOCGNAME(80), &name_of_joystick );

  axis = (int *) calloc( num_of_axis, sizeof( int ) );
  button = (char *) calloc( num_of_buttons, sizeof( char ) );

  printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n"
    , name_of_joystick
    , num_of_axis
    , num_of_buttons
  );

  fcntl( joy_fd, F_SETFL, O_NONBLOCK );	/* use non-blocking mode */


  //============== UART ======================
  int UART = -1;
  //int UART = open( UART_DEVICE, O_RDWR| O_NOCTTY );

  //------------ Try UART devices --------------
  while (UART < 0) {
    UART = open( UART_DEVICE_SAC0, O_RDWR| O_NOCTTY );
    if (UART < 0) {
      printf("ttySAC0 not found");
      UART = open( UART_DEVICE_USB0, O_RDWR| O_NOCTTY );
      if (UART < 0) {
        printf("ttyUSB0 not found");
        UART = open( UART_DEVICE_USB1, O_RDWR| O_NOCTTY );
        if (UART < 0) {
          printf("ttyUSB1 not found");
        } else {
          printf("ttyUSB1 Connected");
          break;
        }
      } else {
        printf("ttyUSB0 Connected");
        break;
      }
    } else {
      printf("ttySAC0 Connected");
      break;
    }

    if(UART >= 0){
      break;
    }

    std::cout << "Retrying in 3 seconds\n";
    sleep(3);

  }
    /*
    try {

      throw "ttySAC0 not found";
    } catch (char const* errmsg){
      printf("%s\n", errmsg);
    }
    */


  /* *** Configure Port *** */
  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( UART, &tty ) != 0 ) {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
  }
  else {
    printf("Attributes Retrieved");
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, B9600);
  cfsetispeed (&tty, B9600);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( UART, TCIFLUSH );
  if ( tcsetattr ( UART, TCSANOW, &tty ) != 0) {
     std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  int resetFlag = 1;

  while(1) {

    if (resetFlag == 1) {
      start = std::clock();
      resetFlag = 0;
    }
    else {

      //tcflush(UART,TCOFLUSH);
  duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

        if (duration > DATA_SPEED) {



           /* read the joystick state */
			read(joy_fd, &js, sizeof(struct js_event));

			/* see what to do with the event */
			switch (js.type & ~JS_EVENT_INIT) {
			  case JS_EVENT_AXIS:
				axis   [ js.number ] = js.value;
				break;
			  case JS_EVENT_BUTTON:
				button [ js.number ] = js.value;
				break;
			}

			/*set values*/
			int ynew = (axis[1]*CONTROL_RANGE)/-32767;
			int rotation = (axis[2]*CONTROL_RANGE)/32767;
			int t_left = (rotation==0? ynew: (rotation/2)+(ynew/2));
			int t_right = (rotation==0? t_left :
				           t_left<0? t_left+CONTROL_RANGE :
				           t_left==0? ynew : t_left-CONTROL_RANGE);

			/*print the results*/
			//printf( "Speed: %3d  Rotation: %3d  ", ynew, rotation);
			//printf("Left Thruster: %3d  Right Thruster: %3d \r", t_left,t_right);

			char cmd[11] = "";
			char right_string[6] = "";

			sprintf(cmd, "%5d", t_left);
			sprintf(right_string, "%5d\r", t_right);
			strcat(cmd, right_string);
	      /* *** WRITE *** */

          //tcflush(UART,TCIFLUSH);
          UART0_Send(UART, cmd, 11);


          printf("%s %s  \r", cmd, right_string);
          fflush(stdout);

        resetFlag = 1;
      }
    }
  }


  close(UART);

}
