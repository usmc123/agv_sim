#include "comm_service.h"
// #include "common_logger.h"
 
using namespace dst_ccms_api;
// using namespace J_COMMON_UTIL;
 
comm_service::comm_service() : m_fd(-1) {}
comm_service::~comm_service() {}
 
int comm_service::CommOpen(char *port)
{
 
    if (commIsOpen())
        return 1;
 
    // m_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    m_fd = open(port, O_RDWR | O_NOCTTY );
    if (m_fd < 0)
    {
        ROS_ERROR("Can't Open Serial Port.");
        return -1;
    }
    //恢复串口为阻塞状态
    if (fcntl(m_fd, F_SETFL, 0) < 0)
    {
        ROS_ERROR("fcntl failed.");
        return -1;
    }
    //测试是否为终端设备
    if (0 == isatty(STDIN_FILENO))
    {
        ROS_ERROR("standard input is not a terminal device.");
        return -1;
    }
    return 1;
}

void comm_service::CommClose()
{
    if (!commIsOpen())
        return;
    close(m_fd);
}
 
int comm_service::commSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity)
{
    if (!commIsOpen())
        return -1;
 
    int i;
    int status;
    int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
 
    struct termios options;
 
    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if (tcgetattr(m_fd, &options) != 0)
    {
        ROS_ERROR("commSet tcgetattr error.");
        return (FALSE);
    }
     bool isStandard = false;
     //设置串口输入波特率和输出波特率
     for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
     {
         if (baudrate == name_arr[i])
         {
             cfsetispeed(&options, speed_arr[i]);
             cfsetospeed(&options, speed_arr[i]);
             isStandard =true;
             break;
         }
     }
 
     if(!isStandard){
         // 非标准波特率
         int res = serial_set_speci_baud(options, baudrate);
         if (res != 0)
         {
             ROS_ERROR("commSet serial_set_speci_baud error.");
             return (FALSE);
        }
     }    
 
    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
 
    //设置数据流控制
    switch (flow_ctrl)
    {
    case 0: //不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: //使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: //使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
 
    //设置数据位,屏蔽其他标志位
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
        ROS_ERROR("Unsupported data size.");
        return (FALSE);
    }
 
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
         options.c_iflag &= ~INPCK;
        options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        break;
    case 'o':
    case 'O': //设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        // options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': //设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        // options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        ROS_ERROR("Unsupported parity.");
        return (FALSE);
    }
 
    // 解决二进制传输时，数据遇到0x0d , 0x11,0x13 会被丢掉的问题   J.  1027
     options.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);
 
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
    options.c_cflag &= ~CSTOPB; //8N1 default config
        ROS_ERROR("Unsupported stop bits.");
        return (FALSE);
    }
 
    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
 
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    //options.c_lflag &= ~(ISIG | ICANON);
 
    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 30;  /* 读取字符需要等待的最少个数为1 */
 
    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    // tcflush(m_fd, TCIFLUSH);
 
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(m_fd, TCSANOW, &options) != 0)
    {
        ROS_ERROR("com set error!");
        return (FALSE);
    }
    return (TRUE);
}
 
int comm_service::CommInit(int speed, int flow_ctrl, int databits, int stopbits, int parity)
{
    
    FD_ZERO(&fs_read);
    
    FD_SET(m_fd, &fs_read);
    
    
    if (!commIsOpen())
        return -1;
    //设置串口数据帧格式
     return commSet(speed, flow_ctrl, databits, stopbits, parity) ;
}
 
int comm_service::CommRecv(char *rcv_buf, int data_len)
{
    // if (!commIsOpen())
    //     return -1;
    // if(rcv_buf==NULL)
    // {return -1;}


    int len, fs_sel;
    // fd_set fs_read;
 
    // FD_ZERO(&fs_read);
    // FD_SET(m_fd, &fs_read);
 
    struct timeval time;
    time.tv_sec = 0;
    time.tv_usec = 4000;
    //使用select实现串口的多路通信
    //  fs_sel = select(m_fd + 1, &fs_read, NULL, NULL, &time);
     
    // if (fs_sel)
    // {
        len = read(m_fd, rcv_buf, data_len);
        return len;
    // }
    // else
    //     return FALSE;
}
 
int comm_service::CommSend(char *send_buf, int data_len)
{
    if (!commIsOpen())
        return -1;
 
    CommFlush(TCIFLUSH);
    int len = 0;
    len = write(m_fd, send_buf, data_len);
    if (len == data_len)
        return len;
    else
    {
        tcflush(m_fd, TCOFLUSH);
        return FALSE;
    }
}
 
void comm_service::CommFlush(int flush)
{
    if (!commIsOpen())
        return;
 
    tcflush(m_fd, flush);
}
 
int comm_service::CommGetFD() const
{
    return m_fd;
}
 
bool comm_service::commIsOpen() const
{
    return m_fd < 0 ? false : true;
}
 
int comm_service::serial_set_speci_baud(struct termios &options,int baud)
{
    struct serial_struct ss,ss_set;
    
    cfsetispeed(&options,B38400);
    cfsetospeed(&options,B38400);
 
    if((ioctl(m_fd,TIOCGSERIAL,&ss))<0){
        ROS_INFO("1.serial_set_speci_baud ioctl.");
        return -1;
    }
 
    ss.flags = ASYNC_SPD_CUST;
    ss.custom_divisor = ss.baud_base / baud;
 
    if((ioctl(m_fd,TIOCSSERIAL,&ss))<0){
        return -1;
    }
 
    ioctl(m_fd,TIOCGSERIAL,&ss_set);
 
    return 0;
}


//crc8 generator polynomial:G(x)=x8+x5+x4+1 



unsigned char comm_service::Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
	unsigned char ucIndex; 
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8 = CRC8_TAB[ucIndex]; 
	} 
	return(ucCRC8); 
}


/* 
** Descriptions: CRC8 Verify function 
** Input: Data to Verify,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
这个函数是验证接收数据的CRC
*/ 
unsigned int comm_service::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) //这里输入的长度是包括CRC校验码的长度
{ 
		unsigned char ucExpected = 0;
		if ((pchMessage == 0) || (dwLength <= 2)) return 0; 
		ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT); 
		return ( ucExpected == pchMessage[dwLength-1] ); 
} 
/* 
** Descriptions: append CRC8 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
将数据设置CRC校验码插入到数据的尾部
*/ 
void comm_service::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) //这里输入的长度是包括CRC校验码的长度
{
	unsigned char ucCRC = 0; 
	if ((pchMessage == 0) || (dwLength <= 2)) 
		return; 
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT); 
	pchMessage[dwLength-1] = ucCRC; 
}




/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t comm_service::Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--) 
	{ 
	chData = *pchMessage++;
	(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; 
	} 
	return wCRC;
}
/* 
** Descriptions: CRC16 Verify function 
** Input: Data to Verify,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
uint32_t comm_service::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) 
{ 
uint16_t wExpected = 0;
if ((pchMessage == NULL) || (dwLength <= 2)) 
{ 
//return __FALSE;
return 0;	
} 
wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT); 
return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == 
pchMessage[dwLength - 1]); 
}
/* 
** Descriptions: append CRC16 to the end of data 
** Input: Data to CRC and append,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
void comm_service::Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) 
{ 
		uint16_t wCRC = 0; 
		if ((pchMessage == NULL) || (dwLength <= 2)) 
		{ 
		return;
		} 
		wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT ); 
		pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff); 
		pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

