/*
sudo sh -c 'echo 100 > /sys/devices/pwm-fan/target_pwm' 쿨링팬 켜기
g++ -o alwasCAMon_test alwasCAMon_test.cpp -lJetsonGPIO -lpthread $(pkg-config opencv4 --libs --cflags)
sudo ./alwasCAMon_test
*/
#include <csignal>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART+
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <JetsonGPIO.h>
#include <pthread.h>    // Used for thread
#include <fstream>
#include <opencv2/opencv.hpp>
#include <errno.h>
#include <string.h>      // Used for soket
#include <arpa/inet.h>   // Used for soket
#include <sys/types.h>   // Used for soket
#include <sys/socket.h>  // Used for soket
#include <netinet/in.h>  // Used for soket
#include "opencv2/opencv.hpp"            //used for opencv
#include "opencv2/imgproc/imgproc.hpp"   //used for opencv
#include "opencv2/highgui/highgui.hpp"   //used for opencv
#include "opencv2/core/core.hpp"         //used for opencv
#include <sys/time.h> // create delay function header


using namespace std;
using namespace cv;
using namespace dnn;
using namespace GPIO;

#define LED 24
#define PIR_FRONT 26
#define CDS 19
int boar_flag = 0;
int elk_flag = 0;
int IR_flag = 0;
VideoCapture cap1(0);
VideoCapture cap2(1);
int boar_sound_count=0; 
int elk_sound_count=0; 
int ir_sound_count=0;

void GPIOsetup();
void signalingHandler(int signo); //Ctrl + C 시그널 명렁함수

// soket setting
#define BUFF_SIZE 1024
int client_socket; 
struct sockaddr_in server_addr;
char buff_rcv[BUFF_SIZE];
char buff_send[BUFF_SIZE];


// serial variable init
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200 // serial baudrate
const char *uart_target = "/dev/ttyTHS1";
int fid = -1;
unsigned char tx_buffer;
char rx_buffer[BUFF_SIZE];


// sec_delay setting
#define ON 		1
#define OFF		0
struct timeval current_time, diff_time, read_start_t;
int timer_on_off;		// Timer On Off Flag
int t_break;
long time_sub(long prev_t, long cur_t ) {
	return cur_t - prev_t;
}

void sec_delay(int t) // t sec delay
{
    timer_on_off = OFF;		// Timer On Off Flag
    do
    {
		if ( timer_on_off == OFF ) 
        {	
			// Get Read Start Time 
			gettimeofday(&read_start_t, NULL);	
			timer_on_off = ON;					
		}				
		gettimeofday(&current_time, NULL);
		diff_time.tv_sec = time_sub(read_start_t.tv_sec, current_time.tv_sec);
								
        if ( diff_time.tv_sec >= t ) // t sec delay
        {    
            timer_on_off = OFF;
            t_break = 0;
        }
        else t_break=1;
    }while(t_break);
}

//int web_escape = 0; //사람이면 이중반복문을 빠져나갈 수 있게 하는 변수
//int IR_escape = 0;



extern "C" void *predater(void*)
{
    system("canberra-gtk-play -f tiger8.wav");
    return NULL;
}

extern "C" void *siren(void*)
{
    system("canberra-gtk-play -f siren.wav");
    return NULL;
}
extern "C" void *re(void*)
{
    sec_delay(3);
	while(1)
	{   
        
		if(read(client_socket,buff_rcv,strlen(buff_rcv)+1)>=0)
        {
            tx_buffer = *buff_rcv;
            write(fid, &tx_buffer, 1);
		    printf("server : %s\n",&tx_buffer);
        }
	}
}

extern "C" void *get(void*)
{
    sec_delay(3);
    while(1)
    {   
        if(read(fid, rx_buffer,strlen(rx_buffer)+1)>=0) 
        {
            
            if(*rx_buffer=='q')
            {
                /*
                if(tx_buffer=='s')
                {
                    strcpy(buff_send,"A harmful wild boar appeared in the field. Click http://192.168.0.9:1880/ui to confirm");	
			        write(client_socket, buff_send, strlen(buff_send)+1);
                }
                else;*/
                sec_delay(4);
                write(fid, &tx_buffer, 1);
            }
            else;
        }
        else ;
    }
    return NULL;
}

int main(void)
{
    GPIOsetup();
    signal(SIGINT, signalingHandler);

    Mat image;
    Mat frame, Big_frame, Gray_frame;
    //Mat dx, dy;
    //Mat fmag, mag;
    //Mat edge;
    Mat Bin;
    Mat dst2;
    Mat labels, stats, centroids, box;
    char savefile[200];        // 이미지 파일  이름을 200자 이내로 제한하기 위한 char 변수 선언
    int day_night = input(CDS); // CDS INPUT variable

    
    // serial setup
    // serial setting
    struct termios  port_options;   // Create the structure                          
    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port 
    fid = open(uart_target, O_RDWR | O_NONBLOCK );
	tcflush(fid, TCIFLUSH);
 	tcflush(fid, TCIOFLUSH);
    sec_delay(1);
    if (fid == -1)
	{
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity   
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size             
    port_options.c_cflag |=  CS8;               // Set the data bits = 8                                 	 
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control                         
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines       				
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            
    port_options.c_oflag &= ~OPOST;                           // No Output Processing
    port_options.c_lflag = 0;               //  enable raw input instead of canonical,
    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly 
    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed 
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed 
    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nJetsno serial OK\n");
    }
    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
    sec_delay(1);
	tcflush(fid, TCIOFLUSH);
    sec_delay(1);
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) |  O_RDWR);

    cout<<"모드를 선택하세요 \r\n순찰 : n, 정지 : s, 뒤집기 : t\r\n"<<endl;

    // soket setting
	client_socket = socket(PF_INET, SOCK_STREAM, 0); //socket()을 이용하여 소켓 생성
	if(-1 == client_socket)
	{
		printf("socket create error\n");
		exit(1);
	}
	memset(&server_addr, 0, sizeof(server_addr)); //메모리의 크기를 변경할 포인터, 초기화 값, 초기화 길이
	server_addr.sin_family = AF_INET;
	server_addr.sin_port=htons(60000);
	server_addr.sin_addr.s_addr=inet_addr("192.168.0.9");
	if(connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1){
		printf("socket create error\n");
		exit(1);}


    //Mat img = imread("boartest.jpg");
    pthread_t p_thread[4];      // 쓰레드 ID
    int thread_id = 0;

    thread_id = pthread_create(&p_thread[0], NULL, re, NULL); // soket read()
    if(thread_id < 0)
	{
		perror("thread create error : ");
		exit(0);
	}
    thread_id = pthread_create(&p_thread[4], NULL, get, NULL); // soket write()
    if(thread_id < 0)
	{
		perror("thread create error : ");
		exit(0);
	}
    
    string class_name;
    std::vector<std::string> classes;
    std::ifstream file("boar_human_elk.names");
    std::string line;

    while (std::getline(file, line)) {
        classes.push_back(line);
    }

    Net net = readNetFromDarknet("yolov4-tiny-custom.cfg", "weights2/yolov4-tiny-custom_last.weights");

    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);

    tx_buffer='n'; //시작은 순찰 모드
    write(fid, &tx_buffer, 1);

    // soket setting
	memset(&server_addr, 0, sizeof(server_addr)); //메모리의 크기를 변경할 포인터, 초기화 값, 초기화 길이
	server_addr.sin_family = AF_INET;
	server_addr.sin_port=htons(60000);
	server_addr.sin_addr.s_addr=inet_addr("192.168.0.9");
    FILE* file_soket =NULL;
    file_soket = fopen("image.jpg","rb");

    while(1)
    {
        if(day_night==0) //웹캠 작동 (낮)
        {
            while (cap1.isOpened()) 
            {
                
                day_night = input(CDS); // CDS 값 확인
                
                cap1 >> image;
                bool isSuccess = cap1.read(image); // image 객체에 저장되는지 확인
                if(!isSuccess){
                    cout << "Could not load the image!" << endl;
                    break;
                }

                auto start = getTickCount();
                
                DetectionModel model = DetectionModel(net);
                model.setInputParams(1 / 255.0, Size(512, 512), Scalar(), true);
                
                std::vector<int> classIds;
                std::vector<float> scores;
                std::vector<Rect> boxes;
                model.detect(image, classIds, scores, boxes, 0.55, 0.45);
                //detect(입력 이미지, 결과 감지의 클래스 인덱스, 해당 신뢰값 집합, 경계 상자값 집합, 임계값, 비임계값)
                
                auto end = getTickCount();
                
                for (int i = 0; i < classIds.size(); i++) 
                {
                    //여기는 그냥 객체가 몇 개에 따라 classIds.size가 달라짐
                    //ex)아무 것도 없으면 for문 동작x, 객체 하나: 0, 객체 둘: 1... 객체 n: n-1
                    rectangle(image, boxes[i], Scalar(0, 255, 0), 2);
                    
                    char text[100];
                    snprintf(text, sizeof(text), "%s: %.2f", classes[classIds[i]].c_str(), scores[i]);
                    //0번째 객체에 해당 객체의 클래스 정보와 신뢰값을 대입, 1번째 객체에 해당 ~~
                    //cout << scores[i] << endl;
                    putText(image, text, Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1,
                    Scalar(0, 255, 0), 2);
                    
                    if(classIds[i] == 1) // if object is human >> n(순찰)
                    {
                        cout << "detected human" << endl;
                        tx_buffer='n'; //n 단어 보냄
                    }
                    else if(classIds[i] <= 0) // if object is boar >> s(정지)
                    {
                        tx_buffer='s'; //s 단어 보냄 퇴치기 멈춤!
                        if((boar_sound_count++)>=10)
                        {
                            thread_id = pthread_create(&p_thread[1], NULL, predater, NULL); //퇴치동작 
                            if(thread_id < 0)
                            {
                                perror("thread create error : predater");
                                exit(1);
                            }
                            boar_sound_count=0;
                        }
                        
                    }
                    else // if object is elk >> s(정지)
                    {
                        tx_buffer='s'; //s 단어 보냄
                        if((elk_sound_count++)>=10)
                        {
                            thread_id = pthread_create(&p_thread[2], NULL, siren, NULL);
                            if(thread_id < 0)
                            {
                                perror("thread create error : siren");
                                exit(1);
                            }
                            elk_sound_count=0;
                        }
                        
                    }

                }
                auto totalTime = (end - start) / getTickFrequency();
                    
                putText(image, "FPS" + to_string(int(1 / totalTime)), Point(50,50), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 2, false);
                imshow("Image", image);
                
                //  파일저장
                sprintf(savefile, "image.jpg");
                imwrite(savefile, image);     // img를 파일로 저장한다.

                // soket file stream
                if(socket(PF_INET, SOCK_STREAM, 0)==-1) //socket()을 이용하여 소켓 생성)
                {
                    printf("socket create error\n");
                    exit(1);
                }
                if(connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1) // 소켓 연결
                {
                    printf("socket create error\n");
                }
                size_t fsize=0, nsize = 0;
                //soket transfer
                fseek(file_soket, 0, SEEK_END);
                fsize=ftell(file_soket);
                fseek(file_soket, 0, SEEK_SET);
                while(nsize!=fsize) {
                    
                    int fpsize = fread(buff_send, 1, 1024, file_soket);
                    nsize += fpsize;
                    send(client_socket, buff_send, fpsize, 0);
                    
                }
                close(client_socket); //소켓 종료


                int k = waitKey(10);
                if (k == 113)break;
                if(day_night==1) // if 조도센서가 밤으로 바뀔경우 while (cap1.isOpened()) 탈출
                {
                    break;
                }

            }
        } 
        else if(day_night == 1)
        {
            
            while (cap2.isOpened()) 
            {
                day_night = input(CDS);

                cap2 >> frame; //영상처리를 위해 캠 영상을 영상 프레임에 주입

                resize(frame, Big_frame, Size(500, 500)); //열화상 카메라 resize
                cvtColor(Big_frame, Gray_frame, COLOR_BGR2GRAY); //영상을 그레이스케일로 변경

                threshold(Gray_frame, Bin, 220, 255, THRESH_BINARY); //이진화를 하여 열 정도가 175를 넘으면 하얀색으로 표시

                int cnt = connectedComponentsWithStats(Bin, labels, stats, centroids); //햐얀 영역의 가로 세로를 판단하기 위함

                for(int i = 1; i < cnt; i++)
                {
                    int* p = stats.ptr<int>(i);

                    if(p[4] < 100) continue; //햐얀 픽셀 수가 50개 이하인 객체는 객체 취급x

                    if(p[2]<p[3]) //객체의 세로가 가로보다 더 길 때 object is human >> n(순찰)
                    {
                        p[2] *= 3; //사람 비율을 정해야할 것 같다 *3을 뭘로할지 생각해보자
                        if((p[2]/p[3])<1) //객체의 세로가 가로보다 더 길 때(사람으로 취급)
                        {
                            tx_buffer='n'; //n 단어 보냄
                            usleep(5000);
                            output(LED, 0);
                            p[2] /= 3;      
                        }
                        else
                        {
                            p[2] /= 3;
                            output(LED, 0);
                        }
                    }
                    else if(p[2] > p[3]) //객체의 가로가 세로보다 더 길 때 object is boar or elk >> s(정지)
                    {
                        p[3] -= 30;
                        if((p[2]/p[3])>=2) 
                        {
                            tx_buffer='s'; //s 단어 보냄 퇴치기 멈춤!
                            if((ir_sound_count++)>=10)
                            {
                                thread_id = pthread_create(&p_thread[1], NULL, predater, NULL); //퇴치동작 
                                if(thread_id < 0)
                                {
                                    perror("thread create error : predater");
                                    exit(1);
                                }
                                ir_sound_count=0;
                            }
                            usleep(5000);
                            output(LED, 1);  
                            p[3] += 30;
                        }
                        else
                        {
                            p[3] += 30;
                            output(LED, 0);
                        }
                    }
                   
                    else
                    {
                        tx_buffer='n'; //n 단어 보냄
                        output(LED, 0);
                    }
                    rectangle(Gray_frame, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
                    rectangle(Bin, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);

                    morphologyEx(Bin, dst2, MORPH_OPEN, Mat());

                    imshow("OPEN", dst2);
                     //  파일저장
                sprintf(savefile, "image.jpg");
                imwrite(savefile, dst2);     // img를 파일로 저장한다.

                // soket file stream
                if(socket(PF_INET, SOCK_STREAM, 0)==-1) //socket()을 이용하여 소켓 생성)
                {
                    printf("socket create error\n");
                    exit(1);
                }
                if(connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1) // 소켓 연결
                {
                    printf("socket create error\n");
                }
                size_t fsize=0, nsize = 0;
                //soket transfer
                fseek(file_soket, 0, SEEK_END);
                fsize=ftell(file_soket);
                fseek(file_soket, 0, SEEK_SET);
                while(nsize!=fsize) {
                    
                    int fpsize = fread(buff_send, 1, 1024, file_soket);
                    nsize += fpsize;
                    send(client_socket, buff_send, fpsize, 0);
                    
                }
                close(client_socket); //소켓 종료


                    waitKey(10);
                }
                
                if(day_night == 0) // if 조도센서가 밤으로 바뀔경우 while (cap2.isOpened()) 탈출
                {
                    break;
                }
            }
        }
    }
}

void GPIOsetup()
{
    setwarnings(false);

    setmode(GPIO::BCM);
    setup(PIR_FRONT, GPIO::IN);
    setup(CDS, GPIO::IN);
    setup(LED, GPIO::OUT);
}

void signalingHandler(int signo) 
{
    printf("\n\nGoodbye World\n\n");
    close(fid);
    cap1.release();
    cap2.release();
    cleanup();
    exit(signo);
}