/*
 * Compile with: g++ -o tinyGPS-test -lrt TinyGPS.cpp main.cpp
 */

#include <stdio.h>
#include "TinyGPS.h"

TinyGPS gps(true);

main(int argc, char **argv)
{
    int c;
    int year;
    byte  month, day, hour, minute, second, hundredths;
    unsigned long time_fix_age = 0;   // last time fix as milliseconds

    while((c = getchar()) != EOF)
    {
        //putchar(c);
        if (!gps.encode(c))
            continue;

        gps.crack_datetime(&year, &month, &day,
                           &hour, &minute, &second,
                           &hundredths, &time_fix_age);

        {
            char sz[120];
            sprintf(sz, "time_fix_age: %ld   Time: %02d/%02d/%04d %02d:%02d:%02d",
                    time_fix_age,
                    day, month, year, hour, minute, second);
            printf("\n");
            printf(sz);
            printf("\n");
        }
    }
}
