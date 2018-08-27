/********************************************************************************* 
 * 
 *       Filename:  ZHYDZD.c
 *    Description:  function:socket、serial、modem-serial 
 *                  
 *        Version:  1.0.0(03/05/2018) 
 *         Author:  YF
 *      ChangeLog:  2018/5/25 修改Server_start() Client_start()为有返回值
 *                  
 ********************************************************************************/
// linux 下读取大于2GB文件时，需指定
#define _FILE_OFFSET_BITS 64

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<sys/socket.h>
#include<sys/ioctl.h>
#include<fcntl.h>
#include<termios.h>
#include<pthread.h>
#include<net/if.h>
#include<netinet/in.h>
#include<arpa/inet.h>

#define CPORT 9020
#define LOCALPORT 7777  
#define SIP "192.168.3.58"
#define DEVICE1 "/dev/ttyS1"//control uart
#define DEVICE2 "/dev/ttyS2"//modem uart
#define DEVICE3 "/dev/ttyS3"//RS232 uart  
//定义包大小10KB和1kB(上位机定义)
#define PACK_SIZE_TCP 1024*100
#define PACK_SIZE_UART 1024
/*define*/
int cli_fd;//上位机
int uart_fd1;//control uart signal
int uart_fd2;//RS232
int uart_fd3;//modem
int sockSev_fd;//server
static int TCP_flag_server = 0;
static int TCP_flag_client = 0;
static int PSTN_flag = 0;
static int UART_flag = 0;
static int socket_flag = 0;
//TCP线路线程
pthread_t thread1,thread2;
pthread_t socktouart;//socket to uart
pthread_t uarttosock;//socket to uart
pthread_t rec, sen;//modem 
pthread_t server_pthread;//server
//网络中断线程终止标识
static char *ctrl_channel;
static char *ctrl_data;

/*标准波特率设置*/    
static int speed_arr[] = { B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1800, B1200, B600, B300};    
static int name_arr[] = {230400, 115200,  57600, 38400, 19200,  9600, 4800,  2400,  1800, 1200,  600, 300};  

/*function*/
int socket_cli(char *ip);
int open_dev(char *dev);
int uart_set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int uart_send(int fd,char *send_buf,int data_len);
int uart_recv(int fd,char *buff,int data_len);
void *Server_start(void *arg);
void *pthread_sertocli(void *arg);
void *pthread_clitoser(void *arg);
int Client_start(char *ip);
void *pthread_clitocli(void *arg);
void *pthread_clitocli_fd(void *arg);
void *pthread_stou(void *arg);//RS232
void *pthread_utos(void *arg);//RS232
int Modem_answer(void);
int Modem_call(char *num);
void *pthread_utos_m(void *arg);
void *pthread_stou_m(void *arg);
void *pthread_Ctrluart(void *arg);
void *pthread_KAB_IP(void *arg);

int main(void)
{
    int su1 = 1;//socket to uart 
    int su2 = 1;
    int ser_return = 1;
    int slock = 1;
    int flock = 0;
    char msg_DT_su[32] = "\n综合路由连接成功！\n";
    char msg_DT_fa[32] = "\n综合路由连接失败！\n";
    char msg_ctrluart_su[32] = "\n控制串口打开成功！\n";
    char msg_ctrluart_fa[32] = "\n控制串口打开失败！\n";
    pthread_t ctrl_listen;
    pthread_t kfb_ip;
    char *dev1 = DEVICE1;
    char *dev2 = DEVICE2;
    char *dev3 = DEVICE3;
    char *IP = SIP;
	
    for(;;)
    {
        uart_fd1 = open_dev(dev1);//open control uart
        if(uart_fd1 > 0)
        {
            uart_set(uart_fd1,115200,0,8,1,'N');//control uart init/set baud rate
            printf("Open contrl uart succeed!\n");//测试指示
            //write(uart_fd1,msg_ctrluart_su,sizeof(msg_ctrluart_su));//发送消息给上位机
            //建立命令串口监听线程
            int cp = pthread_create(&ctrl_listen,NULL,pthread_Ctrluart,&uart_fd1);
            if (cp < 0)
            {
            	/* code */
            	perror("pthread_Ctrluart");
            }
        }
        else
        {
            printf("Open contrl uart failed,try again...\n");//测试指示
            write(uart_fd1,msg_ctrluart_fa,sizeof(msg_ctrluart_fa));//发送消息给上位机
            sleep(1);
            continue;
        }
        while(1)//循环连接
        {
            system("clear");//调试清屏
            cli_fd = socket_cli(IP);//connect da tang server
            if(cli_fd == -1)
            {
                slock = 1;
                sleep(2);
                write(uart_fd1,msg_DT_fa,sizeof(msg_DT_fa));
                continue;
            }
            else
            {   
                //发送本机IP
                int kfb = pthread_create(&kfb_ip,NULL,pthread_KAB_IP,&uart_fd1);
                if (kfb < 0)
                {
                    /* code */
                    perror("pthread_KAB_IP");
                }
                //send  msg to ctrl uart
                if(slock)
                {
                    //write(uart_fd1,msg_DT_su,sizeof(msg_DT_su));
                    slock = 0;
                }
                sleep(1);
                while(cli_fd > 0 && uart_fd1 > 0)//入口条件
                {
                    system("clear");
                    printf(">****************************************************<\n\n");
                    printf("                  ");
                    printf("\033[43;35m欢迎登录综合路由系统\033[0m\n\n");
                    printf(">****************************************************<\n");
                    
                    if (*(ctrl_channel) == '0')
                    {
                        printf("切换TCP线路\n");
                        /*
                        * USB to tcp
                        * need ip
                        * 1.In this department you should connect server or client first
                        * 2.Analyzing conditions is ip
                        */
                        if(*(ctrl_data) == '\0')//server
                        {
                        //start server
                            //Server_start();
                            if(ser_return == 1)
                            {
                                ser_return = pthread_create(&server_pthread,NULL,&(Server_start),NULL);
                            }
                            if(ser_return != 0)
                            {
                                perror("pthread_server"); 
                            }
                            pthread_join(server_pthread,NULL);
                            ser_return = 1;
                        }
                        else
                        {
                        //start client
                            Client_start(ctrl_data);
                        }
                        sleep(1);
                    }
                    else if (*(ctrl_channel) == '1')
                    {
                        printf("切换电话线路\n");
                        /*uart to modem
                        * 1.call target number
                        * 2.if connect succeed,then you can write or read msg
                        */
                        uart_fd3 = open_dev(dev2);
                        uart_set(uart_fd3,115200,0,8,1,'N');
                        /*answer*/
                        if (*(ctrl_data) == '\0') 
                        {
                            /* code */
                            int ma = Modem_answer();
                            printf("ma = %d\n",ma);
                        }
                        /*call*/
                        else
                        {
                            /* code */
                            int mc = Modem_call(ctrl_data);
                            printf("mc = %d\n",mc);
                        }
                    }
                    else if (*(ctrl_channel) == '2')
                    {
                        printf("切换RS232线路\n");
                        /*RS232
                        * 1.open uart
                        * 2.then you can read or write
                        */
                        uart_fd2 = open_dev(dev2);
                        uart_set(uart_fd2,115200,0,8,1,'N');
                        printf("RS232 open succeed!\n");
                        //创建socket到uart线程
                        if(su1 == 1)
                        {
                            su1 = pthread_create(&socktouart,NULL,&(pthread_stou),&uart_fd2);
                        }
                        if(su1 != 0)
                        {
                            perror("pthread_stou"); 
                        }
                        //创建uart到socket线程
                        if(su2 == 1)
                        {
                            su2 = pthread_create(&uarttosock,NULL,&(pthread_utos),&uart_fd2);
                        }
                        if(su2 != 0)
                        {
                            perror("pthread_utos");
                        }
                        UART_flag = 1;
                        pthread_join(socktouart,NULL);
                        pthread_join(uarttosock,NULL);
                        su1 = 1;
                        su2 = 1;
                        sleep(1);
                    }
                    else
                    {
                    	printf("No input!\n");
                        sleep(1);
                    }
                }
            }
        }    
    }
}

/************************************************************************************** 
 *  Description: socket init/相当于client经大唐路由连接服务器
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/ 
int socket_cli(char *ip)
{
    int c_fd;
    struct sockaddr_in serv_addr;
    //create socket
    c_fd = socket(AF_INET,SOCK_STREAM,0);
    if(c_fd == -1)
    {
        perror("scoket cli");
        close(c_fd);
        return -1;
    }
    //init client infor
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(CPORT);
    if((inet_pton(AF_INET,ip,&serv_addr.sin_addr)) < 0)
    {
        perror("inet_pton");
        close(c_fd);
        return -1;
    }
    //connect server
    if(connect(c_fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) == -1)
    {
        perror("connect DaTang");
        close(c_fd);
        return -1;
    }  

    return c_fd;
}
/*
 * open device 
 */ 
int open_dev(char *dev)
{
    int fd;
    fd = open(dev,O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd<0)
    {
        perror("open");
        return -1;
    }
    /*恢复串口为阻塞状态 */                              
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return -1;
    }     

    /*测试是否为终端设备*/    
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return -1;
    }

    return fd;
} 
/*************************************************************************************
 * Description:设置串口数据位，停止位和效验位
 *   Input Args: fd         串口文件描述符
 *                speed     串口速度
 *                flow_ctrl   数据流控制
 *                databits   数据位   取值为 7 或者8
 *                stopbits   停止位   取值为 1 或者2
 *                parity     效验类型 取值为N,E,O,,S
 *  Output Args: 正确返回为1，错误返回为0
 * Return Value: 
 **************************************************************************************/
int uart_set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    struct termios options;
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,
    该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
    */
    if( tcgetattr( fd,&options) != 0)
    {
        perror("SetupSerial 1");    
        return -1; 
    }
    
    //设置串口输入波特率和输出波特率
    for ( int i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
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
            return -1; 
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
            return -1; 
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
            return -1;
    }
    
    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    /*ASCII标准的XON和XOFF字符，如果在传输这两个字符的时候就传不过去，需要把软件流控制屏蔽*/
    options.c_iflag &= ~ (IXON | IXOFF | IXANY);
    /*发送字符0X0d的时候，往往接收端得到的字符是0X0a，
    原因是因为在串口设置中c_iflag和c_oflag中存在从NL-CR和CR-NL的映射，
    即串口能把回车和换行当成同一个字符，可以进行如下设置屏蔽之：*/
    options.c_iflag &= ~ (INLCR | ICRNL | IGNCR);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */  
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)  
    {
        perror("com set error!\n");  
        return -1; 
    }
    return 0;     
}

/*************************************************************************************  
 * 名称：uart_send 
 * 功能：像串口发送数据  
 * 入口参数：fd:文件描述符      
 *          send_buf:存放串口发送数据  
 *          data_len:一帧数据的个数  
 * 出口参数：正确返回为1，错误返回为0  
*************************************************************************************/
int uart_send(int fd, char *send_buf,int data_len)    
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
        return -1;    
    }    
}

/**************************************************************************************  
 * 名称：uart_recv 
 * 功能：接收串口数据  
 * 入口参数：fd :文件描述符      
 * rcv_buf: 接收串口中数据存入rcv_buf缓冲区中  
 * data_len:一帧数据的长度  
 * 出口参数：正确返回为1，错误返回为0  
***************************************************************************************/  
int uart_recv(int fd,char *buff,int data_len)
{
    int len,fs_sel;    
    fd_set fs_read;    
       
    struct timeval time;    
       
    FD_ZERO(&fs_read);    
    FD_SET(fd,&fs_read);    
       
    time.tv_sec = 1;    
    time.tv_usec = 0;    
       
    //使用select实现串口的多路通信    
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time); 
    if(fs_sel)    
    {    
        len = read(fd,buff,data_len);    
        //printf("I got it! fs_sel = %d\n",fs_sel);    
        return len;    
    }    
    else    
    {    
        //printf("No data!");    
        return -1;    
    }  
}
/************************************************************************************** 
 *  Description: server
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
void *Server_start(void *arg)
{
    struct sockaddr_in server;
    struct sockaddr_in client;
    int *connfd;
    int p = 1;
    int s = 1;
    int opt = 1;
    char msg_TCP_online[] = "\n专网通信已建立连接！\n";
    char msg_TCP_onconnect[] = "\n等待客户端连接...\n";

    bzero(&server,sizeof(server));   
    
    server.sin_family = AF_INET;                       //指定协议族 
    server.sin_addr.s_addr = htons(INADDR_ANY);        //指定接收消息的机器  INADDR_ANY->listen all
    server.sin_port = htons(LOCALPORT);                //绑定套接字端口

    sockSev_fd = socket(AF_INET,SOCK_STREAM,0);
    if(sockSev_fd < 0)
    {
        perror("sockSev_fd error");
        close(sockSev_fd);
        pthread_exit(0);
    }
    else
    {
        printf("Creat socket succeed!\n");
    }

    setsockopt(sockSev_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));//端口复用,解决bind error:address already in use

    if(bind(sockSev_fd,(struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0)
    {
        perror("bind error");  
        close(sockSev_fd);
        pthread_exit(0);
    }
    else
    {
        printf("bind....\n");
    }

    if(listen(sockSev_fd, 1))        //启用套接字侦听连接，成功返回0
    {  
        perror("Server Listen");
        close(sockSev_fd);   
        pthread_exit(0); 
    }
    else
    {
        printf("listen....\n");
    }
    socket_flag = 1;
    write(uart_fd1,msg_TCP_onconnect,strlen(msg_TCP_onconnect));//告知上位机等待建立连接

    //printf("等待建立连接!\n");
    //建立连接前关闭服务器
    int c=sizeof(client);
    connfd = malloc(sizeof(int));//防止在两个线程中同时操作一个描述符
    *connfd=accept(sockSev_fd,(struct sockaddr *)&client,&c);
    if(*connfd==-1)
    {
        perror("accept");
        close(sockSev_fd);
        pthread_exit(0);
    }
    printf("accept....\n");
    if(p == 1)
        p = pthread_create(&thread1,NULL,&(pthread_sertocli),connfd);
    if(p != 0)
    {
        perror("pthread_sertocli");
        pthread_exit(0);
    }
    if(s == 1)
        s = pthread_create(&thread2,NULL,&(pthread_clitoser),connfd);
    if (s != 0) 
    {
        perror("pthread_clitoser");
        pthread_exit(0);
    }
    TCP_flag_server = 1;
    write(uart_fd1,msg_TCP_online,strlen(msg_TCP_online));//告知上位机建立连接，通信中
    pthread_join(thread1,NULL);
    pthread_cancel(thread2);
    pthread_join(thread2,NULL);
    
    printf("结束了!\n");
    close(sockSev_fd);
    pthread_exit(0);
}
void *pthread_sertocli(void *arg)
{
    printf("start server to client...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_TCP];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = recv(fd,buf,sizeof(buf),0);
        printf("stc recv recv_data = %d\n", recv_data);
        if(recv_data > 0)
        {
            n = send(cli_fd,buf,recv_data,0);
            printf("stc send n = %d\n", n);
            if(n<0)
            {
                perror("write to cli");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("sertocli connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from server");
            break;
        }
    }
    printf("server to client exit!\n");  
    close(fd);//释放主线程分配的存储模块
    pthread_exit(0); 
}
void *pthread_clitoser(void *arg)
{
    printf("start client to server...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_TCP];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = recv(cli_fd,buf,sizeof(buf),0);
        printf("cts recv recv_data = %d\n", recv_data);
        //printf("cts recv buf: %s\n",buf);
        if(recv_data > 0)
        {
            n = send(fd,buf,recv_data,0);
            printf("cts send n = %d\n", n);
            //printf("cts send buf: %s\n",buf);
            if(n<0)
            {
                perror("write to ser");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("clitoser connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from client");
            break;
        }
    }
    printf("client to server exit!\n");  
    close(fd);
    pthread_exit(0); 
}
/************************************************************************************** 
 *  Description: client
 *   Input Args: ip
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
int Client_start(char *ip)
{
    int sockCli_fd;
    struct sockaddr_in client;
    int p = 1;
    int s = 1;
    char msg_TCP_online[] = "\n专网通信已建立连接!\n";
    char msg_TCP_onconnect[] = "\n正在连接服务器...\n";
    char msg_TCP_fail[] = "\n连接服务器失败,请重试!\n";

    bzero(&client,sizeof(client));
    client.sin_family = AF_INET;
    client.sin_port = htons(LOCALPORT);

    if ((inet_pton(AF_INET,ip,&client.sin_addr)) < 0)
    {
        perror("inet_pton");
        return -1;
    }

    sockCli_fd = socket(AF_INET,SOCK_STREAM,0);
    if(sockCli_fd < 0)
    {
        perror("socket");
        close(sockCli_fd);
        return -1;
    }
    write(uart_fd1,msg_TCP_onconnect,strlen(msg_TCP_onconnect));//告知上位机等待建立连接
    if(-1 == connect(sockCli_fd,(struct sockaddr *)&client,sizeof(client)))
    {
        perror("client connect");
        write(uart_fd1,msg_TCP_fail,strlen(msg_TCP_fail));//告知上位机等待建立连接
        close(sockCli_fd);
        return -1;
    }

    printf("Connect server succeed!\n");

    if(p == 1)
        p = pthread_create(&thread1,NULL,&(pthread_clitocli),&sockCli_fd);
    if(p != 0)
    {
        perror("pthread_clitocli");
        return -1;
    }
    if(s == 1)
        s = pthread_create(&thread2,NULL,&(pthread_clitocli_fd),&sockCli_fd);
    if(s != 0)
    {
        perror("pthread_clitocli_fd");
        return -1;
    }

    TCP_flag_client = 1;
    write(uart_fd1,msg_TCP_online,strlen(msg_TCP_online));//告知上位机建立连接，通信中
    pthread_join(thread1,NULL);
    pthread_cancel(thread2);
    pthread_join(thread2,NULL);
    return 0;
}
void *pthread_clitocli(void *arg)
{
    printf("start client to client...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_TCP];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = recv(cli_fd,buf,sizeof(buf),0);
        printf("ctc recv recv_data = %d\n", recv_data);
        if(recv_data > 0)
        {
            n = send(fd,buf,recv_data,0);
            printf("ctc send n = %d\n", n);
            if(n<0)
            {
                perror("write to cli");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("clitocli connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from client");
            break;
        }
    }
    printf("client to client exit!\n");  
    close(fd);
    pthread_exit(0); 
}
void *pthread_clitocli_fd(void *arg)
{
    printf("start client to client_fd...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_TCP];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = recv(fd,buf,sizeof(buf),0);
        printf("ctcd recv recv_data = %d\n", recv_data);
        //printf("ctcd recv buf: %s\n",buf);
        if(recv_data > 0)
        {
            n = send(cli_fd,buf,recv_data,0);
            printf("ctcd send n = %d\n", n);
            //printf("ctcd send buf: %s\n",buf);
            if(n<0)
            {
                perror("write to cli_fd");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("clitocli_fd connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from client");
            break;
        }
    }
    printf("client to client_fd exit!\n");  
    close(fd);
    pthread_exit(0);    
}

/************************************************************************************** 
 *  Description:modem线路摘机应答 
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/ 
int Modem_answer(void)
{
    int at = 0;
    char command_at[] = "AT\r";
    char at_recv[16];
    char answer_at[] = "ATA\r";
    char num_recv[64]; 
    char speed_buf[32];
    char *baud = speed_buf;
    char pakage[] = "size,";
    int cmp_ring = 0;
    int waitTimer = 0;//等待被呼叫15S
    int callModemTime = 0;
    int sendATA = 0;
    int waitConnect = 0;
    int sendAT = 0;
    int getSpeed = 0;
    char msg_PSIN_online[] = "\nPSTN通信已建立连接!\n";
    char msg_PSIN_oncall[] = "\n等待被呼叫...\n";
    char msg_PSIN_callModem[] = "\n调制解调器异常！\n";
    char msg_PSIN_nocall[] = "\n无呼叫！请重试！\n";
    char msg_PSIN_uartAbnormal[] = "\n串口异常！请重试！\n";
    char msg_PSIN_connectRecvBusy[] = "\nBUSY! 请重试！\n";
    char msg_PSIN_connectRecvNoDIA[] = "\nNO DIAL TONE! 请重试！\n";
    char msg_PSIN_connectRecvNoCARR[] = "\nNO CARRIER! 请重试！\n";
    char msg_PSIN_waitOverTime[] = "\n等待超时! 请重试！\n";
    char msg_PSIN_unableToRespond[] = "\n无法应答! 请重试！\n";
    /*接收at反回值*/
    while(uart_fd3 > 0)
    {
         /*send at command*/
        at = uart_send(uart_fd3,command_at,strlen(command_at));
        sleep(2);
        if(at > 0)
        {
            printf("Send at succeed!\n");
            sleep(1);
            sendAT = 0;
            memset(at_recv,0,sizeof(at_recv));
            int len = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
            if(len > 0)
            {
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                callModemTime = 0;
                //判断接收内容
                int cmp_ok = strncasecmp(at_recv+5,"OK",2);
                //判断有无ring——没有则等待被呼叫
                memset(at_recv,0,sizeof(at_recv));
                int call_msg_lock = 1;
                while(cmp_ok == 0)
                {
                    /* code */
                    if(call_msg_lock)
                    {
                        write(uart_fd1,msg_PSIN_oncall,strlen(msg_PSIN_oncall));//告知上位机等待被呼叫
                        call_msg_lock = 0;
                    }
                    int ring = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
                    if (ring > 0) 
                    {
                        /* code */
                        printf("Recv data: %s\n",at_recv);
                        cmp_ring = strncasecmp(at_recv+2,"RING",4);
                        if(cmp_ring == 0)
                        {
                            waitTimer = 0;
                            break;
                        }
                    }
                    else 
                    {
                        /* code */
                        if(waitTimer < 15)
                        {
                            printf("No ring!\n");
                            sleep(1);
                            waitTimer++;
                            continue;
                        }
                        write(uart_fd1,msg_PSIN_nocall,strlen(msg_PSIN_nocall));//告知上位机无呼叫退出
                        close(uart_fd3);
                        return 8;
                    }
                }

                while(cmp_ring == 0)
                {
                    int send_ata = uart_send(uart_fd3,answer_at,sizeof(answer_at));
                    if (send_ata > 0) 
                    {
                        /* code */
                        printf("Send ATA succeed!\n\n");
                        sendATA = 0;
                        sleep(2);
                        //判断返回的内容
                        while(1)
                        {
                            /* code */
                            memset(num_recv,0,sizeof(num_recv));
                            int recv_connect = uart_recv(uart_fd3,num_recv,sizeof(num_recv));
                            if(recv_connect > 0)
                            {
                                printf("num_recv: %s\nlen: %d\n",num_recv,recv_connect);
                                waitConnect = 0;
                                //判断是否连接成功
                                if (strstr(num_recv,"CONNECT") != 0) 
                                {
                                    /* 发给上位机速率 */
                                    memset(speed_buf,0,sizeof(speed_buf));
                                    strncpy(speed_buf,num_recv+10,6);
                                    printf("Best bause: %s\n",speed_buf);
                                    write(uart_fd1,pakage,strlen(pakage));
                                    write(uart_fd1,speed_buf,strlen(speed_buf));
                                    /*创建收发线程*/
                                    int re = pthread_create(&rec,NULL,&(pthread_stou_m),baud);
                                    if(re != 0)
                                    {
                                        perror("pthread_create rec(answer)");
                                    }

                                    int se = pthread_create(&sen,NULL,&(pthread_utos_m),baud);
                                    if (se != 0) 
                                    {
                                        perror("pthread_create sen(answer)");
                                    }
                                    PSTN_flag = 1;
                                    write(uart_fd1,msg_PSIN_online,strlen(msg_PSIN_online));//告知上位机建立连接，通信中
                                    pthread_join(rec,NULL);
                                    pthread_cancel(sen);
                                    pthread_join(sen,NULL);
                                    return 0;
                                }
                                else if(strstr(num_recv,"BUSY") != 0)
                                {
                                    write(uart_fd1,msg_PSIN_connectRecvBusy,strlen(msg_PSIN_connectRecvBusy));
                                    close(uart_fd3);
                                    return 1;
                                }
                                else if(strstr(num_recv,"NO DIAL") != 0)
                                {
                                    write(uart_fd1,msg_PSIN_connectRecvNoDIA,strlen(msg_PSIN_connectRecvNoDIA));
                                    close(uart_fd3);
                                    return 2;
                                }
                                else if(strstr(num_recv,"NO CARR") != 0)
                                {
                                    write(uart_fd1,msg_PSIN_connectRecvNoCARR,strlen(msg_PSIN_connectRecvNoCARR));
                                    close(uart_fd3);
                                    return 3;
                                }
                            }
                            else
                            {
                                printf("No data!(answer)\n");
                                if(waitConnect < 35)
                                {
                                    waitConnect++;
                                    sleep(1);
                                    continue;
                                }
                                write(uart_fd1,msg_PSIN_waitOverTime,strlen(msg_PSIN_waitOverTime));
                                close(uart_fd3);
                                return 6;
                            }
                        }
                    }
                    else 
                    {
                        printf("Answer failed!\n");
                        if(sendATA < 3)
                        {
                            sendATA++;
                            sleep(1);
                            continue;
                        }
                        write(uart_fd1,msg_PSIN_unableToRespond,strlen(msg_PSIN_unableToRespond));
                        close(uart_fd3);
                        return 7;
                    }
                }
            }
            else
            {
                printf("Can't recv OK!(answer)\n");
                if(callModemTime < 5)
                {
                    callModemTime++;
                    sleep(1);
                    continue;
                }
                write(uart_fd1,msg_PSIN_callModem,strlen(msg_PSIN_callModem));//告知上位机错误
                close(uart_fd3);
                return 9;//3次呼叫modem没有反应跳出
            }
        }
        else
        {
            printf("Send at failed!\n");
            if(sendAT < 5)
            {
                sendAT++;
                sleep(1);
                continue;
            }
            write(uart_fd1,msg_PSIN_uartAbnormal,strlen(msg_PSIN_uartAbnormal));//告知上位机串口可能异常
            close(uart_fd3);
            return 10;
        }
    }
}
/************************************************************************************** 
 *  Description:modem线路呼叫
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
int Modem_call(char *num)
{
    int at = 0;
    char command_at[] = "AT\r";
    char at_recv[16];
    char call_at[32] = "ATD";
    char *call_at2 = num;
    char call_at3[] = "\r";
    char num_recv[64]; 
    char connect_buf[64];
    char speed_buf[32];
    char *baud = speed_buf;
    char pakage[] = "size,";
    int ca = 0;
    int sendAT = 0;
    int callModemTime = 0;
    int modemNotOK = 0;
    int waitConnect = 0;
    char msg_PSIN_uartAbnormal[] = "\n串口异常！请重试！\n";
    char msg_PSIN_online[] = "\nPSTN通信已建立连接!\n";
    char msg_PSIN_call[] = "\n呼叫中...\n";
    char msg_PSIN_callModem[] = "\n调制解调器异常！\n";
    char msg_PSIN_callfailed1[] = "\n拨号：\n";
    char msg_PSIN_callfailed2[] = "\n 失败！请重试！\n";
    char msg_PSIN_waitOverTime[] = "\n拨号超时! 请重试！\n";
    char msg_PSIN_connectRecvBusy[] = "\nBUSY! 请重试！\n";
    char msg_PSIN_connectRecvNoDIA[] = "\nNO DIAL TONE! 请重试！\n";
    char msg_PSIN_connectRecvNoCARR[] = "\nNO CARRIER! 请重试！\n";

    /*接收at反回值*/
    strcat(call_at,call_at2);
    strcat(call_at,call_at3);
    printf("call_at: %s\n",call_at);

    while(uart_fd3 > 0)
    {
        /*send at command*/
        at = uart_send(uart_fd3,command_at,strlen(command_at));
        if(at > 0)
        {
            printf("Send at succeed!\n");
            sendAT = 0;//复位
            sleep(1);
            memset(at_recv,0,sizeof(at_recv));
            int len = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
            if(len > 0)
            {
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                callModemTime = 0;//复位
                //判断接收内容
                if (strstr(at_recv,"OK") != 0)
                {
                    modemNotOK = 0;//复位
                    while(1)
                    {
                        int n = uart_send(uart_fd3,call_at,strlen(call_at));
                        if (n > 0) 
                        {
                            /* code */
                            printf("Call %s succeed!\n\n",call_at2);
                            write(uart_fd1,msg_PSIN_call,strlen(msg_PSIN_call));//告知上位机呼叫中
                            sleep(2);
                            //判断返回的内容
                            while(1)
                            {
                                /* code */
                                memset(num_recv,0,sizeof(num_recv));
                                int r = uart_recv(uart_fd3,num_recv,sizeof(num_recv));
                                if(r > 0)
                                {
                                    printf("num_recv: %s\nlen: %d\n",num_recv,r);
                                    waitConnect = 0;//复位
                                    //判断是否连接成功
                                    if (strstr(num_recv,"CONNECT") != 0) 
                                    {
                                        /* code */
                                        memset(speed_buf,0,sizeof(speed_buf));
                                        strncpy(speed_buf,num_recv+10,6);
                                        printf("Best bause: %s\n",speed_buf);
                                        write(uart_fd1,pakage,strlen(pakage));
                                        write(uart_fd1,speed_buf,strlen(speed_buf));//发送包大大小到控制串口
                                        /*创建收发线程*/
                                        int re = pthread_create(&rec,NULL,&(pthread_stou_m),baud);
                                        if(re != 0)
                                        {
                                            perror("pthread_create rec(call)");
                                        }

                                        int se = pthread_create(&sen,NULL,&(pthread_utos_m),baud);
                                        if (se != 0) 
                                        {
                                            perror("pthread_create sen(call)");
                                        }
                                        PSTN_flag = 1;
                                        write(uart_fd1,msg_PSIN_online,strlen(msg_PSIN_online));//告知上位机建立连接，通信中
                                        pthread_join(rec,NULL);
                                        pthread_cancel(sen);
                                        pthread_join(sen,NULL);
                                        return 6;
                                    }
                                    else if(strstr(num_recv,"BUSY") != 0)
                                    {
                                        write(uart_fd1,msg_PSIN_connectRecvBusy,strlen(msg_PSIN_connectRecvBusy));
                                        close(uart_fd3);
                                        return 7;
                                    }
                                    else if(strstr(num_recv,"NO DIAL") != 0)
                                    {
                                        write(uart_fd1,msg_PSIN_connectRecvNoDIA,strlen(msg_PSIN_connectRecvNoDIA));
                                        close(uart_fd3);
                                        return 8;
                                    }
                                    else if(strstr(num_recv,"NO CARR") != 0)
                                    {
                                        write(uart_fd1,msg_PSIN_connectRecvNoCARR,strlen(msg_PSIN_connectRecvNoCARR));
                                        close(uart_fd3);
                                        return 9;
                                    }
                                }
                                else
                                {
                                    printf("No data(Call)!\n");
                                    if(waitConnect < 35)
                                    {
                                        waitConnect++;
                                        sleep(1);
                                        continue;
                                    }
                                    write(uart_fd1,msg_PSIN_waitOverTime,strlen(msg_PSIN_waitOverTime));
                                    close(uart_fd3);
                                    return 5;
                                }
                            }    
                        }
                        else 
                        {
                            printf("Call %s failed!\n",num);//拨号失败
                            write(uart_fd1,msg_PSIN_callfailed1,strlen(msg_PSIN_callfailed1));
                            write(uart_fd1,call_at2,strlen(call_at2));
                            write(uart_fd1,msg_PSIN_callfailed1,strlen(msg_PSIN_callfailed1));
                            close(uart_fd3);
                            return 4;
                        }
                    }
                }
                else                
                {
                    printf("Modem is not OK!(call)\n");
                    if(modemNotOK < 3)
                    {
                        modemNotOK++;
                        sleep(1);
                        continue;
                    }
                    write(uart_fd1,msg_PSIN_callModem,strlen(msg_PSIN_callModem));//告知上位机错误
                    close(uart_fd3);
                    return 3;//3次呼叫modem没有反应跳出
                }
            }
            else
            {
                printf("Can't recv OK!(call)\n");
                if(callModemTime < 5)
                {
                    callModemTime++;
                    sleep(1);
                    continue;
                }
                write(uart_fd1,msg_PSIN_callModem,strlen(msg_PSIN_callModem));//告知上位机错误
                close(uart_fd3);
                return 2;//3次呼叫modem没有反应跳出
            }
        }
        else
        {
            printf("Send at failed!\n");
            if(sendAT < 5)
            {
                sendAT++;
                sleep(1);
                continue;
            }
            write(uart_fd1,msg_PSIN_uartAbnormal,strlen(msg_PSIN_uartAbnormal));//告知上位机串口可能异常
            close(uart_fd3);
            return 1;
        }
    }  
}
/************************************************************************************** 
 *  Description:接收socket数据并通过串口转发 ----- modem线路
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/ 
void *pthread_stou_m(void *arg)
{
    printf("start socket_to_uart_modem...\n");
    char *spd = (char*)arg;
    int speed = atoi(spd);
    char buf[speed];
    int aaa = sizeof(buf);

    while (1)
    {
        memset(buf,0,sizeof(buf));
        int recv_data = recv(cli_fd,buf,sizeof(buf),0);
        printf("stu(m) recv recv_data = %d\n", recv_data);
        if(recv_data > 0)
        {
            int n = write(uart_fd3,buf,recv_data);
            printf("stu(m) send n = %d\n", n);
            if (n < 0)
            {
                perror("write to uart(m)");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("stou connect failed!(m)\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from socket!(m)");
            break;
        }
        usleep(500);
    }
    printf("socket_to_uart_modem exit...\n"); 
    pthread_exit(0);
}
/************************************************************************************** 
 *  Description:接收串口数据并通过socket转发 ------ modem 线路
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
void *pthread_utos_m(void *arg)
{
    printf("start uart_to_socket_modem...\n");
    char *spd = (char*)arg;
    int speed = atoi(spd);
    char buf[speed];
    char buf_protocol[64];
    int onceLock = 1;

    while (1)
    {
        if(onceLock == 1)//先判断有没有收到反馈信息，有就屏蔽掉
        {
            int recv_protocol = read(uart_fd3,buf_protocol,sizeof(buf_protocol));
            if(strstr(buf_protocol,"PROTOCOL") != 0)
            {
                onceLock = 0;
                memset(buf_protocol,0,sizeof(buf_protocol));
                continue;
            }
        }
        memset(buf,0,sizeof(buf));
        int recv_data = read(uart_fd3,buf,sizeof(buf));
        printf("uts(m) read recv_data = %d\n",recv_data);
        //printf("buf: %s\n",buf);
        if(recv_data > 0)
        {
            int n = send(cli_fd,buf,recv_data,0);
            printf("uts(m) send n = %d\n", n);
            if (n < 0)
            {
                perror("write to socket(m)");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("utos connect failed!(m)\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from uart!(m)");      
            break;
        }
        usleep(500);
    }

    printf("uart_to_socket_modem exit...\n");  
    pthread_exit(0);
}
/************************************************************************************** 
 *  Description:接收socket数据并通过串口转发 
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/ 
void *pthread_stou(void *arg)
{
    printf("start socket_to_uart...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_UART];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = recv(cli_fd,buf,sizeof(buf),0);
        printf("stu recv recv_data = %d\n", recv_data);
        if(recv_data > 0)
        {
            n = write(fd,buf,recv_data);
            printf("stu send n = %d\n", n);
            if (n < 0)
            {
                perror("write to uart");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("stou connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from socket");
            break;
        }
            
        sleep(1);
    }
    printf("socket_to_uart exit...\n"); 
    close(fd); 
    pthread_exit(0);
}
/************************************************************************************** 
 *  Description:接收串口数据并通过socket转发 
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
void *pthread_utos(void *arg)
{
    printf("start uart_to_socket...\n");
    int fd = *(int*)arg;
    int recv_data = 0,n = 0;
    char buf[PACK_SIZE_UART];

    while (1)
    {
        memset(buf,0,sizeof(buf));
        recv_data = read(fd,buf,sizeof(buf));
        printf("uts read recv_data = %d\n",recv_data);
        if(recv_data > 0)
        {
            n = send(cli_fd,buf,recv_data,0);
            printf("uts send n = %d\n", n);
            if (n < 0)
            {
                perror("write to socket");
                break;
            }
        }
        else if(recv_data == 0)
        {
            printf("utos connect failed!\n");
            break;
        }
        else if(recv_data < 0)
        {
            perror("read from uart");      
            break;
        }

        sleep(1);
    }

    printf("uart_to_socket exit...\n");  
    close(fd); 
    pthread_exit(0);
}
/************************************************************************************** 
 *  Description:控制串口监听线程
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
void *pthread_Ctrluart(void *arg)
{
	int fd1 = *(int *)arg;
	char recv_buf[32] = {0};
	char ctrlChannel[8] = {0};
	char ctrlData[16] = {0};
    char command_reverse[8] = "+++";
    char command_ath[8] = "ATH0\r";
    char msg_close_phone[] = "\nPSTN网络通信已中断!\n";
    char msg_close_rs232[] = "\nRS232通信已中断!\n";
    char msg_close_tcp[] = "\n专网通信已中断!\n";
    char msg_close_all[] = "\n无通信连接!\n";

	ctrl_channel = ctrlChannel;
	ctrl_data = ctrlData;

	while(1)
	{
		int len = uart_recv(fd1,recv_buf,sizeof(recv_buf));
		if (len > 0)
		{
			/* code */
			printf("Recv data: %s\n len: %d\n",recv_buf,len);
            strncpy(ctrlChannel,recv_buf,1);
            strncpy(ctrlData,recv_buf+2,strlen(recv_buf)-2);
            printf("ctrl_channel = %s  ctrl_data = %s\n",ctrl_channel,ctrl_data);

            //关闭线程
            if (*(ctrl_channel) == '3') 
            {
                /*关闭串口线程*/
                if(UART_flag == 1)
                {
                    pthread_cancel(socktouart);
                    pthread_cancel(uarttosock);
                    write(uart_fd1,msg_close_rs232,strlen(msg_close_rs232));//通知上位机
                    printf("关闭串口通信方式！\n");
                    UART_flag = 0;
                }
                else if(TCP_flag_server == 1 || TCP_flag_client == 1)
                {
                    pthread_cancel(thread1);
                    pthread_cancel(thread2);
                    write(uart_fd1,msg_close_tcp,strlen(msg_close_tcp));//通知上位机
                    printf("关闭TCP通信方式！\n");
                    TCP_flag_server = 0;
                    TCP_flag_client = 0;
                }
                else if(PSTN_flag == 1)
                {
                    //挂断电话流程(hang up)
                    sleep(1);
                    pthread_cancel(sen);
                    pthread_cancel(rec);
                    sleep(2);
                    int ath1 = uart_send(uart_fd3,command_reverse,strlen(command_reverse));
                    sleep(2);
                    if(ath1 > 0)
                    {
                        printf("hang up(+++)!\n");
                        int ath2 = uart_send(uart_fd3,command_ath,strlen(command_ath));
                        if(ath2 > 0)
                        {
                            printf("hang up(ATH0)!\n");
                        }
                    }
                    printf("hang up modem!\n");
                    write(uart_fd1,msg_close_phone,strlen(msg_close_phone));//通知上位机
                    close(uart_fd3);
                    printf("关闭PSTN通信方式！\n");
                    PSTN_flag = 0;
                }
                else
                {
                    if(socket_flag == 1 && TCP_flag_server == 0)
                    {
                        pthread_cancel(server_pthread);
                        close(sockSev_fd);
                        socket_flag = 0;
                    }
                    write(uart_fd1,msg_close_all,strlen(msg_close_all));//通知上位机
                }
                sleep(1);
            }
		}
		else
		{
            memset(recv_buf,0,sizeof(recv_buf));
            memset(ctrlChannel,0,sizeof(ctrlChannel));
            memset(ctrlData,0,sizeof(ctrlData));
		}
		usleep(500);
	}

	printf("pthread_Ctrluart exit...\n"); 
    pthread_exit(0);
}

void *pthread_KAB_IP(void *arg)
{
    int fd = *(int*)arg;
    int inet_sock;
    struct ifreq ifr;
    char ip[32];
    char Lastip[32];
    char *IP = ip;
    char *LASTIP = Lastip;
    char biaozhi[32] = "ip,";

    while(1)
    {
        inet_sock = socket(AF_INET,SOCK_DGRAM,0);
        strcpy(ifr.ifr_name,"eth0");

        if(ioctl(inet_sock,SIOCGIFADDR,&ifr) < 0)
            perror("ioctl");

        IP = inet_ntoa(((struct sockaddr_in*)&(ifr.ifr_addr))->sin_addr);
        //printf("IP: %s\n",IP);
        //printf("LASTIP: %s\n",LASTIP);         

        if(*(LASTIP) != *(IP))
        {
            memset(LASTIP,0,sizeof(LASTIP));
            strcpy(LASTIP,IP);
            //printf("LASTIP: %s\n",LASTIP);
            strcat(biaozhi,IP);
            //printf("biaozhi: %s\n",biaozhi);
            write(fd,biaozhi,strlen(biaozhi));
        }
        memset(IP,0,sizeof(IP));
        sleep(2);
    }
    
}