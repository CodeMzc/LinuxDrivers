#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include "math.h"

int main(int argc, char *argv[])
{
	
	int data[3];
	double angle;
	double Heading;
	int error;
	int fd = open("/dev/hmc5883l", O_RDWR);
    if(fd < 0)
    {
		printf("open file : %s failed !\n", argv[0]);
		return -1;
	}	

	while(1)
	{
		error = read(fd,data,sizeof(data));
		if(error < 0)
		{
		    printf("write file error! \n");
		    close(fd);
		    
		}

		Heading = atan2((double)data[1],(double)data[0])-0.00669;
		if (Heading>2*3.1415926) 
			Heading = Heading - 2*3.1415926;
		if (Heading<0)    
			Heading = Heading + 2*3.1415926;
		
		angle = Heading*180/3.1415926;
		

		
		printf("angle = %lf\n",angle);
		printf("x = %d,y = %d,z = %d\n",data[0],data[1],data[2]);
		sleep(1);
	}

	
}
