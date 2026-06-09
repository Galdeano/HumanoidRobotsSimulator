#ifndef LECTURE_ECRITURE_H_INCLUDED
#define LECTURE_ECRITURE_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "client_UDP.h"

UDP_data D1,D2;
int LOAD_CHECK=0;

static UDP_data load(const char* filepath);
static void Load_check();
static int data_check(UDP_data D);
static void ecriture(const char* filepath,UDP_data D);
static void Clear_data_log();
static UDP_data lecture(const char* filepath);
static void Receive_All_Data(clock_t time_limit);






static UDP_data load(const char* filepath){

    FILE* fichier = NULL;
    fichier = fopen(filepath, "r");
    UDP_data D;
    D.Id=-1;
    D.statut=0;
    D.controle=0;
    if (fichier != NULL)
    {
        int i=1;
        char sla=fgetc(fichier);
        char C =fgetc(fichier);


        int k=0;
        int Max_data=21;
        for(k=0;k<Max_data;k++){

            while (C!=sla){
                i++;

                C=fgetc(fichier);
                if(C==EOF){printf("\n \n    !!! Erreur, mauvais format de fichier ! Probablement trop court !!! \n");
                            exit(1);}
            }

            fseek(fichier,-i,SEEK_CUR);
            char chaine[i];

            fgets(chaine,i,fichier);
            fseek(fichier,1,SEEK_CUR);

            D.val[k]=atof(chaine);
            printf("\n %f",D.val[k]);

            i=0;

        }

        fclose(fichier);

    }
    else{
        printf("\n Erreur à l'ouverture du fichier :");
        printf(filepath);
        exit(1);
    }

    return D;
}

static void Load_check(){
    UDP_data D1=load("limite_basse.txt"),D2=load("limite_haute.txt");
}

static int data_check(UDP_data D){
    if (LOAD_CHECK==0){
        Load_check();
        LOAD_CHECK=1;
    }

    int i=1;
    int l;
    for (l=0;l<21;l++){
        printf ("\n basse :%f teste : %f haute %f",D1.val[l],D.val[l],D2.val[l]);
        if(D.val[l]<D1.val[l]){
            i=0;
        }
        if(D2.val[l]<D.val[l]){
            i=0;
        }
    }
    return i;
}


static void ecriture(const char* filepath,UDP_data D){
    FILE* fichier = NULL;

    fichier = fopen("data.txt", "a");

    if (fichier != NULL)
    {
        //fseek(fichier, 0,SEEK_END);

        fputc('/',fichier);
        int i=0;
        for (i=0;i<21;i++){
            fprintf(fichier,"%f",D.val[i]);
            fputc('/',fichier);
        }
    fputc('\n',fichier);

    //fputs("Thunderstruck",fichier);

    fclose(fichier);
    }
    else
    {
        // On affiche un message d'erreur si on veut
        printf("Impossible d'ouvrir le fichier d'écriture");
        exit(1);
    }
}

static void Clear_data_log(){
    FILE* fichier = NULL;
    fichier = fopen("data.txt", "w+");
    fclose(fichier);
}

static void Add_data_log(UDP_data D){
    ecriture("data.txt",D);
}



static UDP_data lecture(const char* filepath){
    UDP_data D = load(filepath);
    int i = data_check(D);
    if (i) {
        return D;
    }
    else {
        printf("\n \n   Erreur, donnees hors-limite lors du chargement de %c",filepath);
        exit(1);
    }
}

static void Receive_All_Data(clock_t time_limit){
    clock_t t_start, t_cur;
    t_start = clock(),
    t_cur=clock();
    int time_limit_reach=0;

    while (!time_limit_reach){
        if (Receive_Data()){
        ecriture("data.txt",buff_data);
        t_cur=clock();
        if((t_cur-t_start)>time_limit){
            time_limit_reach=1;
        }
        }
    }

}

#endif // LECTURE_ECRITURE_H_INCLUDED
