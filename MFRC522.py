#!/usr/bin/env python3
'''
Created on Sep 12, 2015

@author: Chris Wilkerson
'''

from spi import SPI
from enum import Enum

class MFRC522:
    _spi = None

    # MFRC522 Register Overview as defined by Table 20
    # Page 0: Command and Status
    # Register Name       Addr (hex)      Function
    Reserved00          = 0x00          # reserved for future use    
    CommandReg          = 0x01          # starts and stops command execution
    ComlEnReg           = 0x02          # enable and disable interrupt request control bits
    DivlEnReg           = 0x03          # enable and disable interrupt request control bits
    ComIrqReg           = 0x04          # interrupt request bits
    DivIrqReg           = 0x05          # interrupt request bits
    ErrorReg            = 0x06          # error bits showing the error status of the last command executed
    Status1Reg          = 0x07          # communication status bits
    Status2Reg          = 0x08          # receiver and transmitter status bits
    FIFODataReg         = 0x09          # input and output of 64 byte FIFO buffer
    FIFOLevelReg        = 0x0A          # number of bytes stored in the FIFO buffer
    WaterLevelReg       = 0x0B          # level for FIFO underflow and overflow warning
    ControlReg          = 0x0C          # miscellaneous control registers
    BitFramingReg       = 0x0D          # adjustments for bit-oriented frames
    CollReg             = 0x0E          # bit position of the first bit-collision detected on the RF interface
    Reserved            = 0x0F          # reserved for future use
    
    # Page 1: Command
    Reserved10          = 0x10          # reserved for future use
    ModeReg             = 0x11          # defines general modes for transmitting and receiving
    TxModeReg           = 0x12          # defines transmission data rate and framing
    RxModeReg           = 0x13          # defines reception data rate and framing
    TxControlReg        = 0x14          # controls the logical behavior of the antenna driver pins TX1 and TX2
    TxASKReg            = 0x15          # controls the setting of the transmission modulation
    TxSelReg            = 0x16          # selects the internal sources for the antenna driver 
    RxSelReg            = 0x17          # selects internal receiver settings
    RxThresholdReg      = 0x18          # selects thresholds for the bit decoder
    DemodReg            = 0x19          # defines demodulator settings
    Reserved1A          = 0x1A          # reserved for future use
    Reserved1B          = 0x1B          # reserved for future use
    MfTxReg             = 0x1C          # controls some MIFARE communication transmit parameters
    MfRxReg             = 0x1D          # controls some MIFARE communication receive parameters
    Reserved1E          = 0x1E          # reserved for future use
    SerialSpeedReg      = 0x1F          # selects the speed of the serial UART interface
    
    # Page 2: Configuration
    Reserved20          = 0x20          # reserved for future use
    CRCResultRegMSB     = 0x21          # shows the MSB and LSB values of the CRC calculation
    CRCResultRegLSB     = 0x22
    Reserved23          = 0x23          # reserved for future use
    ModWidthReg         = 0x24          # controls the ModWidth setting
    Reserved25          = 0x25          # reserved for future use
    RFCfgReg            = 0x26          # configures the receiver gain
    GsNReg              = 0x27          # selects the conductance of the antenna driver pins TX1 and TX2 modulation
    CWGsPReg            = 0x28          # defines the conductance of the p-driver output during periods of no modulation
    ModGsPReg           = 0x29          # defines the conductance of the p-driver output during periods of modulation
    TModeReg            = 0x2A          # defines settings for the internal timer
    TPrescalerReg       = 0x2B          
    TReloadRegMSB       = 0x2C          # defines the 16-bit timer reload value
    TReloadRegLSB       = 0x2D
    TCounterValRegMSB   = 0x2E          # shows the 16-bit timer value
    TCounterValRegLSB   = 0x2F
    
    # Page 3: Test Registers
    Reserved30          = 0x30          # reserved for future use
    TestSel1Reg         = 0x31          # general test signal configuration
    TestSel2Reg         = 0x32          # general test signal configuration and PRBS control
    TestPinEnReg        = 0x33          # enables pin output driver on pins D1 to D7
    TestPinValueReg     = 0x34          # defines the values for D1 to D7 when it is used as an I/O bus Table 125 on page 65
    TestBusReg          = 0x35          # shows the status of the internal test bus
    AutoTestReg         = 0x36          # controls the digital self test
    VersionReg          = 0x37          # shows the software version
    AnalogTestReg       = 0x38          # controls the pins AUX1 and AUX2
    TestDAC1Reg         = 0x39          # defines the test value for TestDAC1
    TestDAC2Reg         = 0x3A          # defines the test value for TestDAC2
    TestADCReg          = 0X3B          # shows the value of ADC I and Q channels
    
    
    
    Commands = Enum(Idle               = 0x0, 
                    Mem                = 0x1, 
                    Generate_Random_ID = 0x2, 
                    Calc_CRC           = 0x3,
                    Rransmit           = 0x4, 
                    No_Cmd_Change      = 0x7, 
                    Receive            = 0x8, 
                    Transceive         = 0xC,
                    MF_Authent         = 0xE,
                    Soft_Reset         = 0xF
                    )
    
    Rx_Gain = Enum(Rx_Gain_18dB     = 0x0 << 4,
                   Rx_Gain_23dB     = 0x1 << 4,
                   Rx_Gain_18dB_2   = 0x2 << 4,
                   Rx_Gain_23dB_2   = 0x3 << 4,
                   Rx_Gain_33_dB    = 0x4 << 4,
                   Rx_Gain_38_dB    = 0x5 << 4,
                   Rx_Gain_43_dB    = 0x6 << 4,
                   Rx_Gain_48_dB    = 0x7 << 4
                   )
    
    
    def __init__(self, device):
        _spi = SPI(device)
            
    def write_to_register(self, register, value):
        self._spi.transfer((register << 1) & 0x7E)
        self._spi.transfer(value)
            
    def read_from_register(self, register):
        self._spi.transfer(0x80 | ((register << 1) & 0x7E))
        return self._spi.transfer(0)
        
    def set_register_bit_mask(self, register, mask):
        val = self.read_from_register(register)
        self.write_to_register(register, val | mask)
        
    def clear_register_bit_mask(self, register, mask):
        val = self.read_from_register(register)
        self.write_to_register(register, val & ~mask)
        
    def set_antenna_gain(self, gain):
        self.clear_register_bit_mask(self.RFCfgReg, 0x7<<4) # Clears RxGain[2:0]
        self.set_antenna_gain(self.RFCfgReg, gain & (0x7<<4)) # Set RxGain[2:0]
        
    def get_antenna_gain(self):
        return self.read_from_register(self.RFCfgReg) & (0x7<<4) # Return RxGain[2:0]
        
    def calculate_crc(self, data, timeout=1000):
        self.write_to_register(self.CommandReg, self.Commands.Idle)
        self.clear_register_bit_mask(self.DivIrqReg, 0x04) # Clear the CRCIRq bit
        self.write_to_register(self.FIFOLevelReg, 0x80) # Flush the buffer
        for i in range(len(data)):
            self.write_to_register(self.FIFODataReg, data[i])
        self.write_to_register(self.CommandReg, self.Commands.Calc_CRC)  
              
        i = 0;
        while True:
            byte = self.read_from_register(self.DivIrqReg)
            # DivIrqReg is:
            #  7       6         5          4          3         2       1          0              
            # Set2, Reserved, Reserved, MfinActlRq, reserved, CRCIRq, reserved, reserved
            if byte & 0x4:
                break
            elif i == timeout:
                return -1
            
        self.write_to_register(self.Commands, self.Commands.Idle)
        res = []
        res.append(self.read_from_register(self.CRCResultRegLSB))
        res.append(self.read_from_register(self.CRCResultRegMSB))
        return res
        
    
        
if __name__ =="__main__":
    t = MFRC522("/dev/bus/usb/001/002")
    print(MFRC522._interface)
    