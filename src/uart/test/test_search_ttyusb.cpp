//
// Created by plutoli on 2022-06-18.
//

#include <cstdio>
#include <cstring>
#include <glob.h>

int main()
{
    glob_t tGlob;
    if(0 != glob("/dev/ttyUSB*", GLOB_ERR, NULL, &tGlob))
    {
        printf("failed to load from folder!\n");
    }

    char fileName[256] = "";
    for(int idx=0; idx<tGlob.gl_pathc; idx++)
    {
        strcpy(fileName, tGlob.gl_pathv[idx]);
        printf("(%d:%d) %s is loading...\n", idx+1, tGlob.gl_pathc, tGlob.gl_pathv[idx]);
    }

    globfree(&tGlob);

    return 0;
}