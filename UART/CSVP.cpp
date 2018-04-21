#include "mbed.h"
#include "CSVP.h"

CSVP::CSVP(PinName TX, PinName RX, int bps) : myserial(TX, RX)
{
    myserial.baud(bps);
    myserial.attach(this, &CSVP::_serialEvent, Serial::RxIrq);
    count = 0;
}

void CSVP::_serialEvent()
{
    buf[count] = myserial.getc();
    if(buf[count] != '\n')
        count++;
    else 
    {
        buf[count] = '\0';
        count = 0;
        rcvd[0] = atoi(strtok(buf, ","));
        int i = 1;
        char *tmp;
        do
        {
            tmp = strtok(NULL, ",");
            rcvd[i++] = atoi(tmp);
        }while(tmp != NULL);
    }
}