#ifndef CLIENT_UDP_H_INCLUDED
#define CLIENT_UDP_H_INCLUDED

#if defined (WIN32)
    #include <winsock2.h>
    typedef int socklen_t;
#elif defined (linux)
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket(s) close(s)
    typedef int SOCKET;
    typedef struct sockaddr_in SOCKADDR_IN;
    typedef struct sockaddr SOCKADDR;
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#define PORT 1337

struct UDP_data
{
    int status; // Message status: 0 no message/not ready/not received, 1 control message, 2 report message (local status becomes status of remote controller)
    int control; // Control commands for remote operator: 0 open link, 1 absolute command, 2 request sinusoid
    int Id; // Message ID, if -1 then the values are not used
    short val[21];
    clock_t time_log;
};
typedef struct UDP_data UDP_data;

static int current_control;
static int active_control;
static int Id_act;
static int Id_max;
static UDP_data client_data;
static UDP_data buff_data;
static SOCKET sock;
static SOCKADDR_IN sock_sin;
static int sin_size=sizeof(sock_sin);

static int Ini();
static void End();
static int Send_Order();
static void Set_Order(int int_control, int nb_Id, int tab[21]);
static int Param_connec_locale();
static int Receive_Data();
static void Param_Order(int int_control, int nb_Id);
static void Print_data(UDP_data D);




static void generate_rand_val(){
    int i;
    for(i=0;i<21;i++){
        short B=(short) ((rand()%3)-1);
        short A= rand()/(short)RAND_MAX;
        client_data.val[i]=(A*B);
    }
}
static int Ini() {
#if defined (WIN32)
        WSADATA WSAData;
        return WSAStartup(MAKEWORD(2,2), &WSAData);
    #else
        return 0;
    #endif
    srand(time(NULL));
}
static void End(){

    closesocket(sock);

#if defined (WIN32)
            WSACleanup();
        #endif

}
static int Send_Order(){ // WARNING: CONNECTION MUST BE CONFIGURED BEFORE CALLING Send_Order()
    Id_act=0;
    Id_max=client_data.Id;
    if (client_data.status!=1){
    printf("Warning: message type is not control, verification recommended\n");
    //return 0; // Uncomment if you want to block sending on warning and/or stop the program.
    //exit(1);
    }

    if(sendto(sock,(const char *)&client_data,sizeof(client_data),0,(SOCKADDR *)&sock_sin,sin_size)<0){
        printf("         Failed to send Send_Order\n");
        return 0;
    }
    else{
        //connecte=1; // !!! Uncomment to stop the loop after the first message is sent
        printf("        Order sent \n");

        return 1;
    }

}
static void Set_Order(int int_control, int nb_Id, int tab[21]){
    client_data.control=int_control;
    client_data.Id=nb_Id;
    client_data.status=1;
    int k=0;
    for(k=0;k<21;k++){
		client_data.val[k]=tab[k];
		}
}
static void Param_Order(int int_control, int nb_Id){
    client_data.control=int_control;
    client_data.Id=nb_Id;
    client_data.status=1;
}
static int Param_connec_locale(){
        Ini();
        //  Socket creation
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        //printf("Socket created \n");
       //  Connection configuration
        sock_sin.sin_addr.s_addr =inet_addr("127.0.0.1");//inet_addr("127.0.0.1"); //inet_addr("10.59.145.197");
        sock_sin.sin_family = AF_INET;
        sock_sin.sin_port = htons(PORT);
        printf("socket created in local mode");

#ifdef WIN32
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
#else
        int flags = fcntl(sock, F_GETFL);
        flags |= O_NONBLOCK;
        fcntl(sock, F_SETFL, flags);
#endif

        return sizeof(sock_sin);
}

static int Param_connec(){
        //  Socket creation
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        //printf("Socket created \n");
       //  Connection configuration
        sock_sin.sin_addr.s_addr =inet_addr("10.59.145.197");//inet_addr("127.0.0.1"); //inet_addr("10.59.145.197");
        sock_sin.sin_family = AF_INET;
        sock_sin.sin_port = htons(PORT);
        printf("socket created in local mode");

#ifdef WIN32
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);
#else
        int flags = fcntl(sock, F_GETFL);
        flags |= O_NONBLOCK;
        fcntl(sock, F_SETFL, flags);
#endif

        return sizeof(sock_sin);
}


static int Receive_Data(){
    if((recvfrom(sock, (char *)&buff_data, sizeof(buff_data), 0, (SOCKADDR *)&sock_sin, &sin_size)) < 0){
        return 0;
        }
    else {
        if (buff_data.status!=2){
            printf("Warning: Received data is not report type, verification recommended\n");
            //return 0; // Uncomment if you want to prevent using message with incorrect status
            //exit(1);
            }
        return 1;
        }

}

static void iniclientdata(){
    client_data.status=1;
    client_data.control=0;
    client_data.Id=-1;
    client_data.time_log=clock();
    int k=0;
    for(k=0;k<21;k++){
       client_data.val[k]=(short)0;
    }
}
static void inibuffdata(){
    buff_data.status=0;
    buff_data.control=0;
    buff_data.Id=0;
    buff_data.time_log=clock();
    int k=0;
    for(k=0;k<21;k++){
       buff_data.val[k]=(short)0;
    }
}
static void Print_data(UDP_data D) {
    printf("Message Id : %d \n",D.Id);
    int j;
    printf("Val  ");
    for(j=0;j<21;j++){
        printf("%d \n", D.val[j]);
    }
}


#endif // CLIENT_UDP_H_INCLUDED
