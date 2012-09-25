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
    int statut; //Statut du message 0 pas de message ou message pas prêt à l'envoie ou pas de message recu, 1 un message de controle, 2 un message de bilan (le controle devient le statut de l'opérateur local)
    int controle; //Donne des ordres à l'opérateur à distance, 0 message d'ouverture de la liaison, 1 message d'ordre absolu, 2 demande d'une sinusoide
    int Id; //Numero du message, si -1 alors les val n'ont aucune importance
    short val[21];
    clock_t time_log;
};
typedef struct UDP_data UDP_data;

static int controle_actuel;
static int controle_en_cours;
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
static void Set_Order(int int_controle, int nb_Id, int tab[21]);
static int Param_connec_locale();
static int Receive_Data();
static void Param_Order(int int_controle, int nb_Id);
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
static int Send_Order(){ //ATTENTION, LA CONNECTION DOIT ETRE PARAMETREE AVANT D'UTILISER .Send_Order()
    Id_act=0;
    Id_max=client_data.Id;
    if (client_data.statut!=1){
    printf("Attention, message de type non-controle, verification recommandee\n");
    //return 0; //A decommenter si on veux qu'une alerte empeche l'envoi du message et/ou l'arret du code.
    //exit(1);
    }

    if(sendto(sock,&client_data,sizeof(client_data),0,(SOCKADDR *)&sock_sin,sin_size)<0){
        printf("         Echec de l'envoie Send_Ordre\n");
        return 0;
    }
    else{
        //connecte=1; // !!! Decommenter pour stopper la boucle au premier message envoyé
        printf("        Ordre envoye \n");

        return 1;
    }

}
static void Set_Order(int int_controle, int nb_Id, int tab[21]){
    client_data.controle=int_controle;
    client_data.Id=nb_Id;
    client_data.statut=1;
    int k=0;
    for(k=0;k<21;k++){
		client_data.val[k]=tab[k];
		}
}
static void Param_Order(int int_controle, int nb_Id){
    client_data.controle=int_controle;
    client_data.Id=nb_Id;
    client_data.statut=1;
}
static int Param_connec_locale(){
        Ini();
        //  Création de la socket
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        //printf("Socket cree \n");
       //  Configuration de la connexion
        sock_sin.sin_addr.s_addr =inet_addr("127.0.0.1");//inet_addr("127.0.0.1"); //inet_addr("10.59.145.197");
        sock_sin.sin_family = AF_INET;
        sock_sin.sin_port = htons(PORT);
        printf("socket cree en mode local");

        int flags = fcntl(sock, F_GETFL);
        flags |= O_NONBLOCK;
        fcntl(sock, F_SETFL, flags);

        return sizeof(sock_sin);
}

static int Param_connec(){
        //  Création de la socket
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        //printf("Socket cree \n");
       //  Configuration de la connexion
        sock_sin.sin_addr.s_addr =inet_addr("10.59.145.197");//inet_addr("127.0.0.1"); //inet_addr("10.59.145.197");
        sock_sin.sin_family = AF_INET;
        sock_sin.sin_port = htons(PORT);
        printf("socket cree en mode local");

        int flags = fcntl(sock, F_GETFL);
        flags |= O_NONBLOCK;
        fcntl(sock, F_SETFL, flags);

        return sizeof(sock_sin);
}


static int Receive_Data(){
    if((recvfrom(sock, &buff_data, sizeof(buff_data), 0, (SOCKADDR *)&sock_sin, &sin_size)) < 0){
        return 0;
        }
    else {
        if (buff_data.statut!=2){
            printf("Attention : Donnee recu de type non-rapport, verification recommandee\n");
            //return 0; //A decommenter si l'on veux empecher l'utilisation du message de mauvais statut
            //exit(1);
            }
        return 1;
        }

}

static void iniclientdata(){
    client_data.statut=1;
    client_data.controle=0;
    client_data.Id=-1;
    client_data.time_log=clock();
    int k=0;
    for(k=0;k<21;k++){
       client_data.val[k]=(short)0;
    }
}
static void inibuffdata(){
    buff_data.statut=0;
    buff_data.controle=0;
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
