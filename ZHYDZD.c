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
#include<fcntl.h>
#include<termios.h>
#include<pthread.h>
#include<netinet/in.h>
#include<arpa/inet.h>

#define CPORT 9020
#define LOCALPORT 7777  
#define SIP "192.168.3.58"
#define DEVICE1 "/dev/ttyS1"//control uart
#define DEVICE2 "/dev/ttyS2"//modem uart
#define DEVICE3 "/dev/ttyS3"//RS232 uart  
//定义包大小10KB和1kB(上位机定义)
#define PACK_SIZE_TCP 1024*10
#define PACK_SIZE_UART 1024
/*define*/
int cli_fd;//上位机
int uart_fd1;//control uart signal
int uart_fd2;//RS232
int uart_fd3;//modem
//网络中断线程终止标识
static int stu_exit = 1;
static int uts_exit = 1;

/*标准波特率设置*/    
static int speed_arr[] = { B230400, B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1800, B1200, B600, B300};    
static int name_arr[] = {230400, 115200,  57600, 38400, 19200,  9600, 4800,  2400,  1800, 1200,  600, 300};  

/*function*/
int socket_cli(char *ip);
int open_dev(char *dev);
int uart_set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int uart_send(int fd,char *send_buf,int data_len);
int uart_recv(int fd,char *buff,int data_len);
int Server_start(void);
void *pthread_sertocli(void *arg);
void *pthread_clitoser(void *arg);
int Client_start(char *ip);
void *pthread_clitocli(void *arg);
void *pthread_clitocli_fd(void *arg);
void *pthread_stou(void *arg);//RS232
void *pthread_utos(void *arg);//RS232
void *pthread_msg(void *arg);// msg to ctl uart
int Modem_answer(void);
int Modem_call(char *num);
void *pthread_utos_m(void *arg);
void *pthread_stou_m(void *arg);

int main(void)
{
    int len;//read bite length
    int su = 0;//socket to uart 
    int slock = 1;
    int flock = 0;
    int stats = 0;
    char msg_DT_su[] = "连接大唐路由成功！\n";
    char msg_DT_fa[] = "连接大唐路由失败！\n";
    char msg_ctrluart_su[] = "打开控制串口成功！\n";
    char msg_ctrluart_fa[] = "打开控制串口失败！\n";
    char msg_at[] = "请重新拨号或接听！\n";
    char msg_at_cut[] = "连接失败！\n";
    //char *cli_connect_succeed = msg;
    pthread_t socktouart;//socket to uart
    pthread_t uarttosock;//socket to uart
    pthread_t cli_msg;//client connect msg
    char *dev1 = DEVICE1;
    char *dev2 = DEVICE2;
    char *dev3 = DEVICE3;
    char *IP = SIP;
    char rcv_buf[50] = {0};
    char ctrlrecv[64] = {0};
    char ctrlcommand[8] = {0};
    char ctrldata[16] = {0};

    /*connect da tang and open contrl uart*/
    for(;;)
    {
        uart_fd1 = open_dev(dev2);//open control uart
        if(uart_fd1)
        {
            uart_set(uart_fd1,115200,0,8,1,'N');//control uart init/set baud rate
            printf("Open contrl uart succeed!\n");
            write(uart_fd1,msg_ctrluart_su,sizeof(msg_ctrluart_su));
        }
        else
        {
            printf("Open contrl uart failed,try again...\n");
            write(uart_fd1,msg_ctrluart_fa,sizeof(msg_ctrluart_fa));
            sleep(1);
            continue;
        }
        while(uart_fd1 > 0)//循环连接
        {
            system("clear");//调试清屏
            cli_fd = socket_cli(IP);//connect da tang server
            if(cli_fd == -1)
            {
                slock = 1;
                sleep(1);
                write(uart_fd1,msg_DT_fa,sizeof(msg_DT_fa));
                continue;
            }
            else
            {   
                //send  msg to ctrl uart
                if(slock)
                {
                    //pthread_create(&cli_msg,NULL,&(pthread_msg),(void *)cli_connect_succeed);
                    write(uart_fd1,msg_DT_su,sizeof(msg_DT_su));
                    slock = 0;
                }
                while(cli_fd)
                {
                    system("clear");
                    printf(">****************************************************<\n\n");
                    printf("                  ");
                    printf("\033[43;35m欢迎登录综合路由系统\033[0m\n\n");
                    printf(">****************************************************<\n");
                    //read contrl uart
                    memset(rcv_buf,0,sizeof(rcv_buf));
                    len = uart_recv(uart_fd1,rcv_buf,32);
                    if(len>0)
                    {
                        rcv_buf[len] = '\0';
                        printf("Recv data is %s  len = %d\n",rcv_buf,len);
                    }
                    else
                    {
                        printf("No data!\n");
                    }
                    strncpy(ctrlrecv,rcv_buf,strlen(rcv_buf));
                    strncpy(ctrlcommand,ctrlrecv,1);//通道
                    strncpy(ctrldata,ctrlrecv+2,strlen(ctrlrecv)-2);//方式
                    printf("ctrlcommand = %s  ctrldata = %s\n",ctrlcommand,ctrldata);
                    if (*(ctrlcommand) == '0')
                    {
                        /*
                        * USB to tcp
                        * need ip
                        * 1.In this department you should connect server or client first
                        * 2.Analyzing conditions is ip
                        */
                        if(*(ctrldata) == '\0')//server
                        {
                        //start server
                            Server_start();
                        }
                        else
                        {
                        //start client
                            Client_start(ctrldata);
                        }
                    }
                    else if (*(ctrlcommand) == '1')
                    {
                        /*uart to modem
                        * 1.call target number
                        * 2.if connect succeed,then you can write or read msg
                        */
                        uart_fd3 = open_dev(dev1);
                        uart_set(uart_fd3,115200,0,8,1,'N');

                        /*answer*/
                        if (*(ctrldata) == '\0') 
                        {
                            /* code */
                            int ma = Modem_answer();
                            if (ma == -1)
                            {
                                /* code */
                                write(uart_fd1,msg_at,sizeof(msg_at));
                            }
                            else
                            {
                                /* code */
                                write(uart_fd1,msg_at_cut,sizeof(msg_at_cut));
                            }

                        }
                        /*call*/
                        else
                        {
                            /* code */
                            Modem_call(ctrldata);

                        }

                    }
                    else if (*(ctrlcommand) == '2')
                    {
                        /*RS232
                        * 1.open uart
                        * 2.then you can read or write
                        */
                        uart_fd2 = open_dev(dev3);
                        uart_set(uart_fd2,115200,0,8,1,'N');
                        printf("RS232 open succeed!\n");
                        //创建socket到uart线程
                        su = pthread_create(&socktouart,NULL,&(pthread_stou),&uart_fd2);
                        if(su < 0)
                        {
                            perror("pthread_stou"); 
                        }
                        //创建uart到socket线程
                        su = pthread_create(&uarttosock,NULL,&(pthread_utos),&uart_fd2);
                        if(su < 0)
                        {
                            perror("pthread_utos");
                        }

                        pthread_join(socktouart,NULL); // 等待线程退出  
                        pthread_join(uarttosock,NULL);

                        close(uart_fd2);
                        printf("RS232 closed!\n");

                    }
                    else
                        printf("Wrong input!\n");
                        //break;
                    memset(ctrlrecv,0,sizeof(ctrlrecv));
                    memset(ctrldata,0,sizeof(ctrldata));
                    memset(ctrlcommand,0,sizeof(ctrlcommand));
                }
            }
        }
    }    
    
    return 0;
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

    //printf("Connect server succeed!");
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
    //else
    //{
        //printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    //}
    /*测试是否为终端设备*/    
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return -1;
    }
    /*else
    {
        printf("isatty success!\n");
    }*/              
    //printf("fd->open=%d\n",fd);

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
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//我加的
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
    //printf("fs_sel = %d\n",fs_sel);    
    if(fs_sel)    
    {    
        len = read(fd,buff,data_len);    
        printf("I got it! fs_sel = %d\n",fs_sel);    
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
int Server_start(void)
{
    int sockSev_fd;
    struct sockaddr_in server;
    struct sockaddr_in client;
    int connfd = 0;
    pthread_t thread1,thread2;

    bzero(&server,sizeof(server));   
    
    server.sin_family = AF_INET;                       //指定协议族 
    server.sin_addr.s_addr = htons(INADDR_ANY);        //指定接收消息的机器  INADDR_ANY->listen all
    server.sin_port = htons(LOCALPORT);                //绑定套接字端口

    sockSev_fd = socket(AF_INET,SOCK_STREAM,0);
    if(sockSev_fd < 0)
    {
        perror("sockSev_fd error");
        close(sockSev_fd);
        return -1;
    }
    else
    {
        printf("Creat socket succeed!\n");
    }

    //setsockopt(sockSev_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if(bind(sockSev_fd,(struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0)
    {
        perror("bind error");  
        close(sockSev_fd);
        return -1;
    }
    else
    {
        printf("bind....\n");
    }

    if(listen(sockSev_fd, 1))        //启用套接字侦听连接，成功返回0
    {  
        perror("Server Listen");
        close(sockSev_fd);   
        return -1;  
    }
    else
    {
        printf("listen....\n");
    }

    while (1)
    {
        int c=sizeof(client);
        connfd=accept(sockSev_fd,(struct sockaddr *)&client,&c);
        if(connfd==-1)
        {
            perror("accept");
            close(connfd);
            close(sockSev_fd);
            return -1;
        }
        printf("accept....\n");
        int p = 0;
        p = pthread_create(&thread1,NULL,&(pthread_sertocli),&connfd);
        if(p < 0)
        {
            perror("pthread_sertocli");
            return -1;
        }
        else
        {
            printf("pthread_sertocli create...\n");
        }

        int s = 0;
        s = pthread_create(&thread2,NULL,&(pthread_clitoser),&connfd);
        if (s < 0) 
        {
            perror("pthread_clitoser");
            return -1;
        }
        else
        {
            printf("pthread_clitoser create...\n");
        }
        pthread_join(thread1,NULL);
        pthread_join(thread2,NULL);
        break;
    }
    close(connfd);
    close(sockSev_fd);
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
        //printf("stc recv buf: %s\n",buf);
        if(recv_data > 0)
        {
            n = send(cli_fd,buf,recv_data,0);
            printf("stc send n = %d\n", n);
            //printf("stc send buf: %s\n",buf);
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
    pthread_t thread1,thread2;
    struct sockaddr_in client;

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

    if(-1 == connect(sockCli_fd,(struct sockaddr *)&client,sizeof(client)))
    {
        perror("client connect");
        close(sockCli_fd);
        return -1;
    }

    printf("Connect server succeed!\n");

    int p = 0;
    p = pthread_create(&thread1,NULL,&(pthread_clitocli),&sockCli_fd);
    if(p < 0)
    {
        perror("pthread_clitocli");
        return -1;
    }
    int s = 0;
    s = pthread_create(&thread2,NULL,&(pthread_clitocli_fd),&sockCli_fd);
    if(s < 0)
    {
        perror("pthread_clitocli_fd");
        return -1;
    }
    pthread_join(thread1,NULL);
    pthread_join(thread2,NULL);
    close(sockCli_fd);
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
        //printf("ctc recv buf: %s\n",buf);
        if(recv_data > 0)
        {
            n = send(fd,buf,recv_data,0);
            printf("ctc send n = %d\n", n);
            //printf("ctc send buf: %s\n",buf);
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
    int an = 0;
    pthread_t rec, sen;

    /*接收at反回值*/
    int aa = 0;
    while(uart_fd3 && an == 0 && aa < 3)
    {
         /*send at command*/
        at = uart_send(uart_fd3,command_at,strlen(command_at));
        if(at > 0)
        {
            printf("Send at succeed!\n");
            memset(at_recv,0,sizeof(at_recv));
            int len = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
            if(len > 0)
            {
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                //判断接收内容
                int cmp = strncasecmp(at_recv+5,"OK",2);
                //判断有无ring——没有则等待被呼叫
                memset(at_recv,0,sizeof(at_recv));
                while(cmp == 0)
                {
                    /* code */
                    int ring = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
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
                while(cmp == 0 && ii < 3 && an == 0)
                {
                    int n = uart_send(uart_fd3,answer_at,sizeof(answer_at));
                    
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
                            int r = uart_recv(uart_fd3,num_recv,sizeof(num_recv));
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
                                    an = strlen(speed_buf);
                                    printf("Best bause: %s\n",speed_buf);
                                    write(uart_fd1,speed_buf,sizeof(speed_buf));//发送包大大小到控制串口
                                    /*创建收发线程*/
                                    int re = pthread_create(&rec,NULL,&(pthread_stou_m),baud);
                                    if(re < 0)
                                    {
                                        perror("pthread_create rec(answer)");
                                    }

                                    int se = pthread_create(&sen,NULL,&(pthread_utos_m),baud);
                                    if (se < 0) 
                                    {
                                        perror("pthread_create sen(answer)");
                                    }
                                    
                                    pthread_join(rec,NULL);
                                    pthread_join(sen,NULL);
                                    break;
                                }
                                else 
                                {
                                    /* code */
                                    printf("Connect failed!(answer)\n");
                                }
                                
                            }
                            else
                            {
                                printf("No data!(answer)\n");
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
                printf("Can't recv data!(answer)\n");
            }
            aa++;
            sleep(1);
        }
        else
        {
            printf("Send at failed!\n");
            sleep(1);
            if(aa < 3)
            {
                aa++;
                continue;
            }
            return -1;
        }   
    }
    
    if (an != 0) {
        /* code */
        printf("连接成功!\n");
    }
    else {
        /* code */
        printf("连接失败!\n");
    }
    close(uart_fd3);
    return 0;
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
    char speed_buf[32];
    char *baud = speed_buf;
    int ca = 0;
    pthread_t rec, sen;

    /*接收at反回值*/
    strcat(call_at,call_at2);
    strcat(call_at,call_at3);
    printf("call_at: %s\n",call_at);

    int cc = 0;
    while(uart_fd3 && ca == 0 && cc < 3)
    {
        /*send at command*/
        at = uart_send(uart_fd3,command_at,strlen(command_at));
        if(at > 0)
        {
            printf("Send at succeed!\n");
            memset(at_recv,0,sizeof(at_recv));
            int len = uart_recv(uart_fd3,at_recv,sizeof(at_recv));
            if(len > 0)
            {
                printf("Recv data: %s\ndata len: %d\n",at_recv,len);
                //判断接收内容
                int cmp = strncasecmp(at_recv+5,"OK",2);
                if (cmp != 0) 
                {
                    /* code */
                    printf("Modem is not OK!\n(call)");
                }
                else 
                {
                    /* code */
                    int ii = 0;
                    while(ii < 3 && ca == 0)
                    {
                        int n = uart_send(uart_fd3,call_at,sizeof(call_at));
                        if (n > 0) 
                        {
                            /* code */
                            printf("Call %s succeed!\n\n",num);

                            //判断返回的内容
                            int nn = 0;
                            while(n && nn < 8)
                            {
                                /* code */
                                memset(num_recv,0,sizeof(num_recv));
                                int r = uart_recv(uart_fd3,num_recv,sizeof(num_recv));
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
                                        ca = strlen(speed_buf);
                                        printf("Best bause: %s\n",speed_buf);
                                        write(uart_fd1,speed_buf,sizeof(speed_buf));//发送包大大小到控制串口
                                        /*创建收发线程*/
                                        int re = pthread_create(&rec,NULL,&(pthread_stou_m),baud);
                                        if(re < 0)
                                        {
                                            perror("pthread_create rec(call)");
                                        }

                                        int se = pthread_create(&sen,NULL,&(pthread_utos_m),baud);
                                        if (se < 0) 
                                        {
                                            perror("pthread_create sen(call)");
                                        }
                                        
                                        pthread_join(rec,NULL);
                                        pthread_join(sen,NULL);
                                        break;
                                    }
                                    else 
                                    {
                                        /* code */
                                        printf("Connect failed!(call)\n");
                                    }
                                    
                                }
                                else
                                {
                                    printf("No data(Call)!\n");
                                }
                                nn++;
                                sleep(1);
                            }
                            
                        }
                        else 
                        {
                            /* code */
                            printf("Call %s failed!\n",num);
                        }
                        ii++;
                        sleep(1);
                    }
                }
            }
            else
            {
                /* code */
                printf("Can't recv data!\n(call)");
            }
            cc++;
            sleep(1);
        }
        else
        {
            printf("Send at failed!\n");
            sleep(1);
            if(cc < 3)
            {
                cc++;
                continue;
            }
            return -1;
        }
    }
    
    if (ca != 0) {
        /* code */
        printf("连接成功!\n");
    }
    else {
        /* code */
        printf("连接失败!\n");
    }
    close(uart_fd3);
    return 0;    
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
            //break;
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
    }
    printf("stu_exit = %d\n",stu_exit);
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

    while (1)
    {
        memset(buf,0,sizeof(buf));
        int recv_data = read(uart_fd3,buf,sizeof(buf));
        printf("uts(m) read recv_data = %d\n",recv_data);
        if(recv_data > 0)
        {
            int n = send(cli_fd,buf,recv_data,0);
            printf("uts(m) send n = %d\n", n);
            if (n < 0)
            {
                perror("write to socket(m)");
                break;
            }
            //break;
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
            //break;
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
    }
    printf("stu_exit = %d\n",stu_exit);
    printf("socket_to_uart exit...\n");  
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
            //break;
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
    }

    printf("uart_to_socket exit...\n");  
    pthread_exit(0);
}
/************************************************************************************** 
 *  Description:收集信息发送给控制串口
 *   Input Args: 
 *  Output Args: 
 * Return Value: 
 *************************************************************************************/
void *pthread_msg(void *arg)
{
    //printf("start pthread_msg\n");
    char *str;
    str = arg;
    int n = 0;

    n = write(uart_fd1,str,32);
    if(n<0)
    {
        perror("Send state msg Error");
    }
    printf("Send state msg!\n");
    pthread_exit(0);
}
