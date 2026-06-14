#ifndef READ_WRITE_H_INCLUDED
#define READ_WRITE_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "client_UDP.h"

UDP_data D1,D2;
int LOAD_CHECK=0;

static UDP_data load(const char* filepath);
static void Load_check();
static int data_check(UDP_data D);
static void write_data(const char* filepath, UDP_data D);
static void Clear_data_log();
static UDP_data read_data(const char* filepath);
static void Receive_All_Data(clock_t time_limit);


static UDP_data load(const char* filepath){

    FILE* file = NULL;
    file = fopen(filepath, "r");
    UDP_data D;
    D.Id=-1;
    D.status=0;
    D.control=0;
    if (file != NULL)
    {
        int i=1;
        char sla=fgetc(file);
        char C =fgetc(file);


        int k=0;
        int Max_data=21;
        for(k=0;k<Max_data;k++){

            while (C!=sla){
                i++;

                C=fgetc(file);
                if(C==EOF){
                    printf("\n \n    !!! Error, bad file format! Probably too short !!! \n");
                    exit(1);
                }
            }

            fseek(file,-i,SEEK_CUR);
            char buffer[i];

            fgets(buffer,i,file);
            fseek(file,1,SEEK_CUR);

            D.val[k]=atof(buffer);
            printf("\n %f",D.val[k]);

            i=0;

        }

        fclose(file);

    }
    else{
        printf("\n Error opening file: ");
        printf("%s", filepath);
        exit(1);
    }

    return D;
}

static void Load_check(){
    D1=load("limite_basse.txt");
    D2=load("limite_haute.txt");
}

static int data_check(UDP_data D){
    if (LOAD_CHECK==0){
        Load_check();
        LOAD_CHECK=1;
    }

    int i=1;
    int l;
    for (l=0;l<21;l++){
        printf ("\n low: %f test: %f high: %f", D1.val[l], D.val[l], D2.val[l]);
        if(D.val[l]<D1.val[l]){
            i=0;
        }
        if(D2.val[l]<D.val[l]){
            i=0;
        }
    }
    return i;
}


static void write_data(const char* filepath, UDP_data D){
    FILE* file = NULL;

    file = fopen("data.txt", "a");

    if (file != NULL)
    {
        fputc('/',file);
        int i=0;
        for (i=0;i<21;i++){
            fprintf(file,"%f",D.val[i]);
            fputc('/',file);
        }
        fputc('\n',file);

        fclose(file);
    }
    else
    {
        printf("Unable to open data file for writing");
        exit(1);
    }
}

static void Clear_data_log(){
    FILE* file = NULL;
    file = fopen("data.txt", "w+");
    fclose(file);
}

static void Add_data_log(UDP_data D){
    write_data("data.txt",D);
}



static UDP_data read_data(const char* filepath){
    UDP_data D = load(filepath);
    int i = data_check(D);
    if (i) {
        return D;
    }
    else {
        printf("\n \n   Error, data out-of-bounds while loading %s", filepath);
        exit(1);
    }
}

static void Receive_All_Data(clock_t time_limit){
    clock_t t_start, t_cur;
    t_start = clock();
    t_cur=clock();
    int time_limit_reach=0;

    while (!time_limit_reach){
        if (Receive_Data()){
            write_data("data.txt",buff_data);
            t_cur=clock();
            if((t_cur-t_start)>time_limit){
                time_limit_reach=1;
            }
        }
    }

}

#endif // READ_WRITE_H_INCLUDED
