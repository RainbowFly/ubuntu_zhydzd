
/****************************************************************************
* 文 件 名：modem.c
* 功    能：实现两个modem间的拨号/应答连接，传输数据
* 说明：
****************************************************************************/
//串口相关的头文件    
#include<stdio.h>      /*标准输入输出定义*/    
#include<stdlib.h>     /*标准函数库定义*/    
#include<unistd.h>     /*Unix 标准函数定义*/    
#include<sys/types.h>     
#include<sys/stat.h>       
#include<fcntl.h>      /*文件控制定义*/    
#include<termios.h>    /*PPSIX 终端控制定义*/    
#include<errno.h>      /*错误号定义*/    
#include<string.h>   
#include<pthread.h> 

//宏定义    
#define FALSE  -1    
#define TRUE   0 

/*标准波特率设置*/    
static int speed_arr[] = { B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1800, B1200, B600, B300};    
static int name_arr[] = {230400, 115200,  57600, 38400, 19200,  9600, 4800,  2400,  1800, 1200,  600, 300};    

static int fd1;
/*******************************************************************  
 * 名称：UART_Open  
 * 功能：打开串口并返回串口设备文件描述  
 * 入口参数：fd:文件描述符 port:串口号(ttyS0,ttyS1,ttyS2)  
 * 出口参数：正确返回为1，错误返回为0  
*******************************************************************/  
int UART_Open(int fd,char* port)    
{    
       
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);    
    if (FALSE == fd)    
    {    
        perror("Can't Open Serial Port");    
        return(FALSE);    
    }    
    //恢复串口为阻塞状态                                   
    if(fcntl(fd, F_SETFL, 0) < 0)    
    {    
        printf("fcntl failed!\n");    
        return(FALSE);    
    }         
    else    
    {    
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));    
    }    
    //测试是否为终端设备        
    if(0 == isatty(STDIN_FILENO))    
    {    
        printf("standard input is not a terminal device\n");    
        return(FALSE);    
    }    
    else    
    {    
        printf("isatty success!\n");    
    }                  
    //printf("fd->open=%d\n",fd);    
    return fd;    
}

/*******************************************************************  
 * 名称：UART_Close  
 * 功能：关闭串口并返回串口设备文件描述  
 * 入口参数：fd:文件描述符 port:串口号(ttyS0,ttyS1,ttyS2)  
 * 出口参数：void  
*******************************************************************/    
void UART_Close(int fd)    
{    
    close(fd);    
} 

/*******************************************************************  
 * 名称：UART_Set  
 * 功能：设置串口数据位，停止位和效验位  
 * 入口参数：fd 串口文件描述符  
 *          speed      串口速度  
 *          flow_ctrl  数据流控制  
 *          databits   数据位   取值为 7 或者8  
 *          stopbits   停止位   取值为 1 或者2  
 *          parity     效验类型 取值为N,E,O,,S  
 *出口参数：正确返回为1，错误返回为0  
*******************************************************************/ 
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)    
{    

    struct termios options;    
       
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  
    */    
    if( tcgetattr( fd,&options)  !=  0)    
    {    
        perror("SetupSerial 1");        
        return(FALSE);     
    }    
      
    //设置串口输入波特率和输出波特率    
    for ( int i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)    
    {    
        if  (speed == name_arr[i])    
        {                 
            cfsetispeed(&options, speed_arr[i]);     
            cfsetospeed(&options, speed_arr[i]);      
        }    
    }         
       
    //修改控制模式，保证程序不会占用串口    
    options.c_cflag |= CLOCAL;    
    //修改控制模式，使得能够从串口中读取输入数据    
    options.c_cflag |= CREAD;    
      
    //设置数据流控制    
    switch(flow_ctrl)    
    {    
          
        case 0 ://不使用流控制    
            options.c_cflag &= ~CRTSCTS;    
            break;       
        case 1 ://使用硬件流控制    
            options.c_cflag |= CRTSCTS;    
            break;    
        case 2 ://使用软件流控制    
            options.c_cflag |= IXON | IXOFF | IXANY;    
            break;    
    }    
    //设置数据位    
    //屏蔽其他标志位    
    options.c_cflag &= ~CSIZE;    
    switch (databits)    
    {      
        case 5:    
            options.c_cflag |= CS5;    
            break;    
        case 6:    
            options.c_cflag |= CS6;    
            break;    
        case 7:        
            options.c_cflag |= CS7;    
            break;    
        case 8:        
            options.c_cflag |= CS8;    
            break;      
        default:       
            fprintf(stderr,"Unsupported data size\n");    
            return (FALSE);     
    }    
    //设置校验位    
    switch (parity)    
    {      
        case 'n':    
        case 'N': //无奇偶校验位。    
            options.c_cflag &= ~PARENB;     
            options.c_iflag &= ~INPCK;        
            break;     
        case 'o':      
        case 'O'://设置为奇校验        
            options.c_cflag |= (PARODD | PARENB);     
            options.c_iflag |= INPCK;                 
            break;     
        case 'e':     
        case 'E'://设置为偶校验      
            options.c_cflag |= PARENB;           
            options.c_cflag &= ~PARODD;           
            options.c_iflag |= INPCK;          
            break;    
        case 's':    
        case 'S': //设置为空格     
            options.c_cflag &= ~PARENB;    
            options.c_cflag &= ~CSTOPB;    
            break;     
        default:      
            fprintf(stderr,"Unsupported parity\n");        
            return (FALSE);     
    }     
    // 设置停止位     
    switch (stopbits)    
    {      
        case 1:       
            options.c_cflag &= ~CSTOPB; break;     
        case 2:       
            options.c_cflag |= CSTOPB; break;    
        default:       
            fprintf(stderr,"Unsupported stop bits\n");     
            return (FALSE);    
    }    
       
    //修改输出模式，原始数据输出    
    options.c_oflag &= ~OPOST;    
      
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    //options.c_lflag &= ~(ISIG | ICANON);    
       
    //设置等待时间和最小接收字符    
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */      
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */    
       
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读    
    tcflush(fd,TCIFLUSH);    
       
    //激活配置 (将修改后的termios数据设置到串口中）    
    if (tcsetattr(fd,TCSANOW,&options) != 0)      
    {    
        perror("com set error!\n");      
        return (FALSE);     
    }    
    return (TRUE);     
}  

/*******************************************************************  
 * 名称：UART_Recv  
 * 功能：接收串口数据  
 * 入口参数：fd :文件描述符      
 * rcv_buf: 接收串口中数据存入rcv_buf缓冲区中  
 * data_len:一帧数据的长度  
 * 出口参数：正确返回为1，错误返回为0  
*******************************************************************/    
int UART_Recv(int fd, char *rcv_buf,int data_len)    
{    
    int len,fs_sel;    
    fd_set fs_read;    
       
    struct timeval time;    
       
    FD_ZERO(&fs_read);    
    FD_SET(fd,&fs_read);    
       
    time.tv_sec = 10;    
    time.tv_usec = 0;    
       
    //使用select实现串口的多路通信    
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);    
    if(fs_sel)    
    {    
        len = read(fd,rcv_buf,data_len);    
        //printf("I am right! len = %d fs_sel = %d\n",len,fs_sel);    
        return len;    
    }    
    else    
    {    
        //printf("Sorry,I am wrong!");    
        return FALSE;    
    }         
}

/********************************************************************  
 * 名称：UART_Send  
 * 功能：发送数据  
 * 入口参数：fd:文件描述符      
 *          send_buf:存放串口发送数据  
 *          data_len:一帧数据的个数  
 * 出口参数：正确返回为1，错误返回为0  
*******************************************************************/
int UART_Send(int fd, char *send_buf,int data_len)    
{    
    int len = 0;    
       
    len = write(fd,send_buf,data_len);    
    if (len == data_len )    
    {    
        printf("send data is %s\n",send_buf);  
        return len;    
    }         
    else       
    {    
        tcflush(fd,TCOFLUSH);    
        return FALSE;    
    }    
}

void *uart_rec(void *arg)
{
    char recv_test[64];

    int ii = 0;
    while(1 && ii<20)
    {
        memset(recv_test,0,sizeof(recv_test));
        UART_Recv(fd1,recv_test,sizeof(recv_test));
        ii++;
    }
    exit(1);
    
}

void *uart_sen(void *arg)
{
    char *a = (char*)arg;
    printf("a : %s\n",a);
    int t = atoi(a);
    char send_test[t] = {0};


    for(int i = 0; i < 10; i++)
    {
        /* code */
        UART_Send(fd1,send_test,strlen(send_test));
        sleep(1);
    }
    exit(1);
}

int main(int argc, char **argv)
{
    /* code */

    int err;
    int num;
    int at;
    char command_at[] = "AT\r";
    char at_recv[16];
    char call_num[] = "ATD603\r";
    char answer_at[] = "ATA\r";
    char num_recv[64];
    char speed_buf[32];
    char *baud = speed_buf;
    int s = 0;//call
    int a = 0;//answer
    pthread_t rec, sen;
    //1.打开串口
    fd1 = UART_Open(fd1,argv[1]);
    do{
        err = UART_Set(fd1,115200,0,8,1,'N');
    }while(err == FALSE || fd1 == FALSE);

    //2.主被叫选择
    printf("选择呼叫(输入0)或应答(输入1)\n请输入: ");
    scanf("%d",&num);
    while(fd1 && s == 0 && a == 0)
    {
        /*呼叫*/
        if(num == 0)
        {
            //发送at验证modem
            at = UART_Send(fd1,command_at,strlen(command_at));
            if(at>0)
            {
                printf("Send AT succeed!\n\n");
            }
            else
            {
                printf("Send AT failed!\n\n");
                continue;
            }

            sleep(1);
            //接收at返回命令
            memset(at_recv,0,sizeof(at_recv));
            int len = UART_Recv(fd1,at_recv,sizeof(at_recv));
            if (len > 0) 
            {
                /* code */
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                //判断接收内容
                int cmp = strncasecmp(at_recv+5,"OK",2);
                int ii = 0;
                while(cmp == 0 && ii < 3 && s == 0)
                {
                    int n = UART_Send(fd1,call_num,sizeof(call_num));
                    
                    if (n > 0) 
                    {
                        /* code */
                        printf("Call num succeed!\n\n");

                        //判断返回的内容
                        int nn = 0;
                        while(n && nn < 8)
                        {
                            /* code */
                            memset(num_recv,0,sizeof(num_recv));
                            int r = UART_Recv(fd1,num_recv,sizeof(num_recv));
                            if(r > 0)
                            {
                                printf("num_recv: %s\nlen: %d\n",num_recv,r);
                                int c = strncasecmp(num_recv+2,"CONNECT",7);
                                //判断是否连接成功
                                if (c == 0) 
                                {
                                    /* code */
                                    memset(speed_buf,0,sizeof(speed_buf));
                                    strncpy(speed_buf,num_recv+10,strlen(num_recv)-10);
                                    s = strlen(speed_buf);
                                    printf("Best bause: %s\n",speed_buf);

                                    /*创建收发线程*/
                                    int re = pthread_create(&rec,NULL,&(uart_rec),baud);
                                    if(re < 0)
                                    {
                                        perror("pthread_create rec");
                                    }

                                    int se = pthread_create(&sen,NULL,&(uart_sen),baud);
                                    if (se < 0) 
                                    {
                                        perror("pthread_create sen");
                                    }
                                    
                                    pthread_join(rec,NULL);
                                    pthread_join(sen,NULL);
                                    
                                    break;
                                }
                                else 
                                {
                                    /* code */
                                    printf("Connect failed!\n");
                                }
                                
                            }
                            else
                            {
                                printf("No data(num_recv)!\n");
                            }
                            nn++;
                            printf("nn: %d\n",nn);
                            sleep(1);
                        }
                        
                    }
                    else 
                    {
                        /* code */
                        printf("Call num failed!\n");
                    }
                    ii++;
                    sleep(1);
                }
            }
            else 
            {
                /* code */
                printf("Can't recv data!\n");
                //continue;
            }
   
        }
        /*应答*/
        else if(num == 1)
        {
            //发送at验证modem
            at = UART_Send(fd1,command_at,strlen(command_at));
            if(at>0)
            {
                printf("Send AT succeed!\n\n");
            }
            else
            {
                printf("Send AT failed!\n\n");
                continue;
            }

            sleep(1);
            //接收at返回命令
            memset(at_recv,0,sizeof(at_recv));
            int len = UART_Recv(fd1,at_recv,sizeof(at_recv));
            if (len > 0) 
            {
                /* code */
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                //判断接收内容
                int cmp = strncasecmp(at_recv+5,"OK",2);
                //判断有无ring——没有则等待被呼叫
                memset(at_recv,0,sizeof(at_recv));
                while(cmp == 0)
                {
                    /* code */
                    int ring = UART_Recv(fd1,at_recv,sizeof(at_recv));
                    if (ring > 0) 
                    {
                        /* code */
                        printf("Recv data: %s\n",at_recv);
                        int cc = strncasecmp(at_recv+2,"RING",4);
                        if(cc==0)
                        {
                            break;
                        }
                    }
                    else 
                    {
                        /* code */
                        printf("No ring!\n");
                        continue;
                    }
                    sleep(1);
                    
                }
                
                int ii = 0;
                while(ii < 3 && a == 0)
                {
                    int n = UART_Send(fd1,answer_at,sizeof(answer_at));
                    
                    if (n > 0) 
                    {
                        /* code */
                        printf("Answer succeed!\n\n");

                        //判断返回的内容
                        int nn = 0;
                        while(n && nn < 8)
                        {
                            /* code */
                            memset(num_recv,0,sizeof(num_recv));
                            int r = UART_Recv(fd1,num_recv,sizeof(num_recv));
                            if(r > 0)
                            {
                                printf("num_recv: %s\nlen: %d\n",num_recv,r);
                                int c = strncasecmp(num_recv+2,"CONNECT",7);
                                //判断是否连接成功
                                if (c == 0) 
                                {
                                    /* code */
                                    memset(speed_buf,0,sizeof(speed_buf));
                                    strncpy(speed_buf,num_recv+10,strlen(num_recv)-10);
                                    a = strlen(speed_buf);
                                    printf("Best bause: %s\n",speed_buf);
                                    /*创建收发线程*/
                                    int re = pthread_create(&rec,NULL,&(uart_rec),baud);
                                    if(re < 0)
                                    {
                                        perror("pthread_create rec");
                                    }

                                    int se = pthread_create(&sen,NULL,&(uart_sen),baud);
                                    if (se < 0) 
                                    {
                                        perror("pthread_create sen");
                                    }
                                    
                                    pthread_join(rec,NULL);
                                    pthread_join(sen,NULL);

                                    break;
                                }
                                else 
                                {
                                    /* code */
                                    printf("Connect failed!\n");
                                }
                                
                            }
                            else
                            {
                                printf("No data(Answer)!\n");
                            }
                            nn++;
                            sleep(1);
                        }
                        
                    }
                    else 
                    {
                        /* code */
                        printf("Answer failed!\n");
                    }
                    ii++;
                    sleep(1);
                }
            }
            else 
            {
                /* code */
                printf("Can't recv data!\n");
                //continue;
            }

        }
        sleep(1);
        break;
    }
    /**/
    if (s != 0 && a != 0) {
        /* code */
        printf("建立连接!\n");
    }
    else {
        /* code */
        printf("连接失败!\n");
    }
    /*关闭当前波特率串口*/
    UART_Close(fd1);
    return 0;
}
