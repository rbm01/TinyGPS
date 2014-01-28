#include <TinyGPS.h>

TinyGPS tinyGps;

void setup()
{
    Serial1.begin(9600);
    Serial.begin(9600);
}

void loop()
{
    bool rc = false;
    int year;
    byte  month, day, hour, minute, second, hundredths;
    unsigned long time_fix_age, posn_fix_age;   // last time fix as mS

    while (Serial1.available())
    {
        int c = Serial1.read();
        if (c)
        {
            Serial.write(c);
            if (tinyGps.encode(c))
                rc = true;
        }
    }

    if (rc)
    {
        char buf[80];
        tinyGps.crack_datetime(&year, &month, &day,
                               &hour, &minute, &second,
                               &hundredths, &time_fix_age);

        tinyGps.get_position(NULL, NULL, &posn_fix_age);

        Serial.println();
        snprintf_P(buf, sizeof(buf), PSTR("time_fix_age/posn_fix_age: %lu / %lu mS;  GPS Time: %02d/%02d/%04d %02d:%02d:%02d.%02d"),
                   time_fix_age, posn_fix_age,
                   day, month, year, hour, minute, second, hundredths);
        Serial.println(buf);
    }

}
