#include "server.hpp"


int Read_I2C::obj_count = 0;

///////////////////////////////////////////////////////////////////////////////
//  Constructor
//  Initialize object and setup I2C communications
///////////////////////////////////////////////////////////////////////////////
Read_I2C::Read_I2C(int addr){
    if(obj_count==0){
        if(gpioInitialise()<0)
            valid = false;
        else{
            valid = true;
            init_slave(addr);
        }
    }
    else{
        valid = false;
    }
    obj_count++;
}


///////////////////////////////////////////////////////////////////////////////
//  Destructor
//  Make proper termination of communications
///////////////////////////////////////////////////////////////////////////////
Read_I2C::~Read_I2C(){
    obj_count--;
    if(valid){
        Read_I2C::close_slave();
    }
    if(obj_count==0){
        gpioTerminate();
    }
}

///////////////////////////////////////////////////////////////////////////////
//  Read_I2C::init_slave()
//  Setup I2C pin modes, and initialize communication protocol
///////////////////////////////////////////////////////////////////////////////
void Read_I2C::init_slave(int addr){
    gpioSetMode(18, PI_ALT3);
    gpioSetMode(19, PI_ALT3);
    xfer.control = (addr<<16) | /* Slave address */
                   (0x00<<13) | /* invert transmit status flags */
                   (0x00<<12) | /* enable host control */
                   (0x00<<11) | /* enable test fifo */
                   (0x00<<10) | /* invert receive status flags */
                   (0x01<<9) | /* enable receive */
                   (0x01<<8) | /* enable transmit */
                   (0x00<<7) | /* abort and clear FIFOs */
                   (0x00<<6) | /* send control reg as 1st I2C byte */
                   (0x00<<5) | /* send status regr as 1st I2C byte */
                   (0x00<<4) | /* set SPI polarity high */
                   (0x00<<3) | /* set SPI phase high */
                   (0x01<<2) | /* enable I2C mode */
                   (0x00<<1) | /* enable SPI mode */
                   0x01; /* enable BSC peripheral */
    bscXfer(&xfer);
    return;
}

///////////////////////////////////////////////////////////////////////////////
//  Read_I2C::close_slave()
//  End I2C communications
///////////////////////////////////////////////////////////////////////////////
void Read_I2C::close_slave(void) {
    xfer.control = 0;
    bscXfer(&xfer);
    return;
}

///////////////////////////////////////////////////////////////////////////////
//  Read_I2C::read_buff()
///////////////////////////////////////////////////////////////////////////////
bool Read_I2C::read_message(char msg[]){
    if(!valid) return false;

    xfer.txCnt = 0;
    bscXfer(&xfer);

    if(xfer.rxCnt==STD_MSG_LENGHT){
        for(int i=0;i<STD_MSG_LENGHT;++i)
            msg[i] = xfer.rxBuf[i];
        return true;
    }

    return false;
}
