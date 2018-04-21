#include "mbed.h"
#include "CSV.h"

CSV::CSV(PinName TX, PinName RX, int bps) : myserial(TX, RX)
{
    myserial.baud(bps);
    myserial.attach(this, &CSV::_serialEvent, Serial::RxIrq);
    dataIndex = 0;
}

void CSV::_serialEvent()
{
    rcvData[dataIndex] = myserial.getc();
    if(rcvData[dataIndex] != '\n')
        dataIndex++;
    else
    {
        rcvData[dataIndex] = '\0';
        dataIndex = 0;
        valueData[0] = atoi(strtok(rcvData, ","));
        int i = 1;
        char *tmp;
        do
        {
            tmp = strtok(NULL, ",");
            valueData[i++] = atoi(tmp);
        }while(tmp != NULL);
    }
}

int CSV::getValue(int num)
{
    return valueData[num];
}
