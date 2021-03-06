#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<sys/types.h>
#include<errno.h>

static int ret;
static int fd;

/*
 * 安全读写函数
 */

ssize_t safe_write(int fd, const void *vptr, size_t n)
{
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = vptr;
    nleft = n;

    while(nleft > 0)
    {
    if((nwritten = write(fd, ptr, nleft)) <= 0)
        {
            if(nwritten < 0&&errno == EINTR)
                nwritten = 0;
            else
                return -1;
        }
        nleft -= nwritten;
        ptr   += nwritten;
    }
    return(n);
}

ssize_t safe_read(int fd,void *vptr,size_t n)
{
    size_t nleft;
    ssize_t nread;
    char *ptr;

    ptr=vptr;
    nleft=n;

    while(nleft > 0)
    {
        if((nread = read(fd,ptr,nleft)) < 0)
        {
            if(errno == EINTR)//被信号中断
                nread = 0;
            else
                return -1;
        }
        else
        if(nread == 0)
            break;
        nleft -= nread;
        ptr += nread;
    }
    return (n-nleft);
}

int uart_open(int fd,const char *pathname)
{
    assert(pathname);

    /*打开串口*/
    fd = open(pathname,O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == -1)
    {
        perror("Open UART failed!");
        return -1;
    }

    /*清除串口非阻塞标志*/
    if(fcntl(fd,F_SETFL,0) < 0)
    {
        fprintf(stderr,"fcntl failed!\n");
        return -1;
    }

    return fd;
}

int uart_set(int fd,int baude,int c_flow,int bits,char parity,int stop)
{
    struct termios options;

    /*获取终端属性*/
    if(tcgetattr(fd,&options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }


    /*设置输入输出波特率，两者保持一致*/
    switch(baude)
    {
        case 4800:
            cfsetispeed(&options,B4800);
            cfsetospeed(&options,B4800);
            break;
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 921600:
            cfsetispeed(&options,B921600);
            cfsetospeed(&options,B921600);
            break;
        default:
            fprintf(stderr,"Unkown baude!\n");
            return -1;
    }

    /*设置控制模式*/
    options.c_cflag |= CLOCAL;//保证程序不占用串口
    options.c_cflag |= CREAD;//保证程序可以从串口中读取数据

    /*设置数据流控制*/
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    /*设置数据位*/
    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    /*设置停止位*/
    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    /*设置输出模式为原始输出*/
    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/
    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN] = 1;//最少读取一个字符

    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

int uart_read(int fd,char *r_buf,size_t len)
{
    ssize_t cnt = 0;
    fd_set rfds;
    struct timeval time;

    /*将文件描述符加入读描述符集合*/
    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);

    /*设置超时为15s*/
    time.tv_sec = 15;
    time.tv_usec = 0;

    /*实现串口的多路I/O*/
    ret = select(fd+1,&rfds,NULL,NULL,&time);
    switch(ret)
    {
        case -1:
            fprintf(stderr,"select error!\n");
            return -1;
        case 0:
            fprintf(stderr,"time over!\n");
            return -1;
        default:
            cnt = safe_read(fd,r_buf,len);
            if(cnt == -1)
            {
                fprintf(stderr,"read error!\n");
                return -1;
            }
            return cnt;
    }
}

int uart_write(int fd,const char *w_buf,size_t len)
{
    ssize_t cnt = 0;

    cnt = safe_write(fd,w_buf,len);
    if(cnt == -1)
    {
        fprintf(stderr,"write error!\n");
        return -1;
    }

    return cnt;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    /*可以在这里做些清理工作*/

    return 0;
}

int open_uart(int baud, const char *uart_name, int data_bits, int parity_bits, int stop_bits)
{
    int fd; 
    struct termios t;
    static struct termios _t_bak;

    #ifndef B460800
    #define B460800 460800
    #endif

    #ifndef B921600
    #define B921600 921600
    #endif

    #ifndef B1000000
    #define B1000000 1000000
    #endif

    /* process baud rate */
    int speed;
    int databit, stopbit, paritybit;

    switch(data_bits)
    {
        case 5:
            databit = CS5;
            break;
        case 6:
            databit = CS6;
            break;
        case 7:
            databit = CS7;
            break;
        case 8:
        default:
            databit = CS8;
            break;
    }

    switch(stop_bits)
    {
        case 1:
            stopbit = 1;
            break;
    }
    (void)stopbit;
    (void)paritybit;

    switch (baud) {
    case 0:      speed = B0;      break;

    case 50:     speed = B50;     break;

    case 75:     speed = B75;     break;

    case 110:    speed = B110;    break;

    case 134:    speed = B134;    break;

    case 150:    speed = B150;    break;

    case 200:    speed = B200;    break;

    case 300:    speed = B300;    break;

    case 600:    speed = B600;    break;

    case 1200:   speed = B1200;   break;

    case 1800:   speed = B1800;   break;

    case 2400:   speed = B2400;   break;

    case 4800:   speed = B4800;   break;

    case 9600:   speed = B9600;   break;

    case 19200:  speed = B19200;  break;

    case 38400:  speed = B38400;  break;

    case 57600:  speed = B57600;  break;

    #ifdef B100000
    case 100000: speed = B100000; break;
    #endif

    case 115200: speed = B115200; break;

    case 230400: speed = B230400; break;

    case 460800: speed = B460800; break;

    case 921600: speed = B921600; break;//

    case 1000000: speed = B1000000; break;

    #ifdef B1500000
    case 1500000: speed = B1500000; break;
    #endif

    #ifdef B3000000
    case 3000000: speed = B3000000; break;
    #endif

    default:
       
        return -EINVAL;
    }

    fd = open(uart_name, O_RDWR | O_NONBLOCK);

    // tcsetattr(fd, TCSANOW, &_t_bak);

    memset(&t, 0, sizeof(struct termios));
    cfmakeraw(&t);

    t.c_ispeed = speed;                // set the baud

    t.c_cc[VTIME] = 0;
    t.c_cc[VMIN] = 0;

    t.c_cflag |= (t.c_cflag & ~CSIZE) | databit;                         // set data length
     /*设置停止位*/
    if( stopbit == 1)/*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
    {
        t.c_cflag &= ~CSTOPB;/*默认为一位停止位； */
    }
    else if( stopbit == 2)
    {
        t.c_cflag |= CSTOPB;/*CSTOPB表示送两位停止位*/
    }
    /*设置奇偶校验位*/
     switch( parity_bits )
     {
         case 'O':  /*奇校验*/
             t.c_cflag |= PARENB;/*开启奇偶校验*/
             t.c_iflag |= (INPCK | ISTRIP);/*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
             t.c_cflag |= PARODD;/*启用奇校验(默认为偶校验)*/
             break;
         case 'E':/*偶校验*/
             t.c_cflag |= PARENB; /*开启奇偶校验  */
             t.c_iflag |= ( INPCK | ISTRIP);/*打开输入奇偶校验并去除字符第八个比特*/
             t.c_cflag &= ~PARODD;/*启用偶校验*/
             break;
         case 'N': /*无奇偶校验*/
             t.c_cflag &= ~PARENB;
             break;
     }
    
    int rv = tcsetattr(fd, TCSANOW, &t);
    if(rv < 0) {

        printf("tcsetattr failed\n");
        return -EINVAL;
    }
    printf("uart set\n");
    return fd;
}
int main(void)
{
    const char *w_buf = "something to write";
    size_t w_len = sizeof(w_buf);

    unsigned char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,"/dev/ttyUSB1");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn*/
    // open_uart(921600, "/dev/ttyUSB1", 8, 'N', 1);
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,921600,0,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

    // ret = uart_write(fd,w_buf,w_len);
    // if(ret == -1)
    // {
    //     fprintf(stderr,"uart write failed!\n");
    //     exit(EXIT_FAILURE);
    // }

    while(1)
    {
        ret = read(fd,r_buf, 1000);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }else{
            int k=0;
            printf("ret=%d\n", ret);
            for(k=0;k<ret;k++)
            {
                printf("%x ", r_buf[k]);
            }
            printf("\n");
        }
        usleep(1000);
    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}