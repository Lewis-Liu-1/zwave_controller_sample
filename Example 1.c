
#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions      */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdint.h>

void main(void) {
    int fd; /*File Descriptor*/

    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */



    if (fd == -1) /* Error Checking */
        printf("\n  Error! in Opening ttyACM0  ");
    else
        printf("\n  ttyACM0 Opened Successfully ");


    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings; /* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings); /* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings, B115200); /* Set Read  Speed as B115200                       */
    cfsetospeed(&SerialPortSettings, B115200); /* Set Write Speed as B115200                       */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB; /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB; /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE; /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |= CS8; /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS; /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST; /*No Output Processing*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 10; /* Read at least 10 characters */
    SerialPortSettings.c_cc[VTIME] = 0; /* Wait indefinetly   */


    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none");

    /*------------------------------- Read data from serial port -----------------------------*/

    tcflush(fd, TCIFLUSH); /* Discards old data in the rx buffer            */

    char read_buffer[32]; /* Buffer to store the data received              */
    int bytes_read = 0; /* Number of bytes read by the read() system call */
    int i = 0;

    uint8_t cmds[] = {0x06};
    write(fd, cmds, sizeof (cmds));
    write(fd, cmds, sizeof (cmds));
    write(fd, cmds, sizeof (cmds));

    uint8_t tx_buffer[] = {0x01, 0x03, 0x00, 0x02, 0xfe};

    printf("try to get device list\r\n");
    write(fd, tx_buffer, sizeof (tx_buffer));

    tcdrain(fd); /* delay for output */

    printf("read from device\r\n");
    while (1) {
        bytes_read = read(fd, &read_buffer, 32); /* Read the data                   */
        if (bytes_read == 0) break;

        for (i = 0; i < bytes_read; i++) /*printing only the received characters*/ {
            printf("%02X ", (0xff & read_buffer[i]));
        }
    }

    close(fd); /* Close the serial port */

}