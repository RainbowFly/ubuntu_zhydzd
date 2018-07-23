
static char *ctrl_channel;
static char *ctrl_data;


void *pthread_Ctrluart(void *arg);


void *pthread_Ctrluart(void *arg)
{
	int fd1 = *(int *)arg;
	char recv_buf[32];
	char ctrlChannel[8];
	char ctrlData[16];

	ctrl_channel = ctrlChannel;
	ctrl_data = ctrlData;
	
	while(1)
	{
		memset(recv_buf,0,sizeof(recv_buf));
		memset(ctrlChannel,0,sizeof(ctrlChannel));
		memset(ctrlData,0,sizeof(ctrlData));
		int len = uart_recv(fd1,recv_buf,sizeof(recv_buf));
		if (len > 0)
		{
			/* code */
			printf("Recv data: %s\n len: %d\n",recv_buf,len);
		}
		else
		{
			printf("No data!\n");
		}

		strncpy(ctrlChannel,recv_buf,1);
		strncpy(ctrlData,recv_buf+2,strlen(recv_buf)-2);
		printf("ctrlChannel = %s  ctrlData = %s\n",ctrlChannel,ctrlData);
		sleep(2);

	}

	printf("pthread_Ctrluart exit...\n"); 
    pthread_exit(0);
}
	
int main(void)
{
    int su = 0;//socket to uart 
    int slock = 1;
    int flock = 0;
    char msg_DT_su[] = "连接大唐路由成功！\n";
    char msg_DT_fa[] = "连接大唐路由失败！\n";
    char msg_ctrluart_su[] = "打开控制串口成功！\n";
    char msg_ctrluart_fa[] = "打开控制串口失败！\n";
    char msg_at[] = "请重新拨号或接听！\n";
    char msg_at_cut[] = "连接失败！\n";
    pthread_t socktouart;//socket to uart
    pthread_t uarttosock;//socket to uart
    pthread_t cli_msg;//client connect msg
    pthread_t ctrl_listen;
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
            write(uart_fd1,msg_ctrluart_su,sizeof(msg_ctrluart_su));//发送消息给上位机
            //建立命令串口监听线程
            int cp = pthread_create(&ctrl_listen,NULL,pthread_Ctrluart,&uart_fd1);
            if (cp < 0)
            {
            	/* code */
            	perror("pthread_Ctrluart");
            }
            //等待线程退出（基本不会退出）
            pthread_join(ctrl_listen,NULL);
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
                while(cli_fd > 0 && uart_fd1 > 0)//入口条件
                {
                    system("clear");
                    printf(">****************************************************<\n\n");
                    printf("                  ");
                    printf("\033[43;35m欢迎登录综合路由系统\033[0m\n\n");
                    printf(">****************************************************<\n");
                    
                    if (*(ctrl_channel) == '0')
                    {
                        /*
                        * USB to tcp
                        * need ip
                        * 1.In this department you should connect server or client first
                        * 2.Analyzing conditions is ip
                        */

                        if(ctrl_data == '\0')//server
                        {
                        //start server
                            Server_start();
                        }
                        else
                        {
                        //start client
                            Client_start(ctrl_data);
                        }
                    }
                    else if (*(ctrl_channel) == '1')
                    {
                        /*uart to modem
                        * 1.call target number
                        * 2.if connect succeed,then you can write or read msg
                        */
                        uart_fd3 = open_dev(dev3);
                        uart_set(uart_fd3,115200,0,8,1,'N');

                        /*answer*/
                        if (ctrl_data == '\0') 
                        {
                            /* code */
                            int ma = Modem_answer();
                            if (ma == -1){
                                /* code */
                                write(uart_fd1,msg_at,sizeof(msg_at));
                            }
                            else{
                                /* code */
                                write(uart_fd1,msg_at_cut,sizeof(msg_at_cut));
                            }

                        }
                        /*call*/
                        else
                        {
                            /* code */
                            int mc = Modem_call(ctrl_data);
                            
                            if (mc == -1) {
                                /* code */
                                write(uart_fd1,msg_at,sizeof(msg_at));
                            }
                            else {
                                /* code */
                                write(uart_fd1,msg_at_cut,sizeof(msg_at_cut));
                            }
                        }

                    }
                    else if (*(ctrl_channel) == '2')
                    {
                        /*RS232
                        * 1.open uart
                        * 2.then you can read or write
                        */
                        uart_fd2 = open_dev(dev2);
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
                    else if(*(ctrl_channel) == '3')
                    {
                    	printf("关闭所有通信方式！\n");
                    }
                    else
                    {
                    	printf("Wrong input!\n");
                    	//memset(ctrl_data,0,sizeof(*ctrl_data));
                    	//memset(ctrl_channel,0,sizeof(*ctrl_channel));
                    }
            	}
        	}
    	}    
    }
    return 0;
}