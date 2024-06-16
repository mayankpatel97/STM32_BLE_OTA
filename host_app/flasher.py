import serial
import sys
import os
import time
import struct

FW_TYPE_APP = 0x01
FW_TYPE_BOOTLOADER = 0x02

FW_TYPE = FW_TYPE_APP
#FW_TYPE = FW_TYPE_BOOTLOADER
FW_VERSION = 0x3A67

crc16_table = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]


ACK = 0
NACK = 1
ETX_OTA_DATA_MAX_SIZE = 128
START_BYTE = 0x2A
END_BYTE = 0x23

CMD_START_PACKET = 0x01
CMD_INFO_PACKET = 0x02
CMD_START_PACKET_LENGTH = 0x01 # including cmd,2byte data length and 1 byte data
CMD_FWDATA_PACKET = 0x03
CMD_STOP_PACKET = 0x04
CMD_STOP_PACKET_LENGTH = 0x01



ERROR_CODES = [
    "",
    "ERROR : PACKET NOT ACKNOWLEDGED",
    "ERROR : PACKET RESPONSE TIMEOUT",
    "ERROR : FIRMWARE SIZE IS TOO BIG",
]

def calculate_crc16(data):
    #print("calculating CRC of data :",data)
    crc = 0xFFFF
    for byte in data:
        crc = (crc << 8) ^ crc16_table[((crc >> 8) ^ byte) & 0xFF];
        #crc = (crc >> 8) ^ crc16_table[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF





def ota_send_start_command(port):
    #port.write("Sending OTA START".encode("utf-8"))
    start_packet = []
    start_packet.append(START_BYTE)
    start_packet.append(CMD_START_PACKET)
    start_packet.append(CMD_START_PACKET_LENGTH)
    start_packet.append(0x00)
    start_packet.append(0x01)
    crc16 = calculate_crc16(start_packet[1:])
    crc_byte_array = crc16.to_bytes(2, byteorder='big')
    start_packet.append(crc_byte_array[1])
    start_packet.append(crc_byte_array[0])
    start_packet.append(END_BYTE)
    print("Start Packet : ", start_packet)
    #crc_byte_array = crc16.to_bytes(2, byteorder='big')
    port.write(bytes(start_packet))

def ota_send_header_command(port,fileSize,FW_TYPE,FW_CRC,Version):
    #port.write("Sending OTA Header".encode("utf-8"))
    #CMD_INFO_PACKET
    info_packet = []
    info_packet.append(START_BYTE)
    info_packet.append(CMD_INFO_PACKET)
    info_packet.append(0x09)
    info_packet.append(0x00) # packet length

    filesize_byte_array = fileSize.to_bytes(4, byteorder='big')
    info_packet.append(int(filesize_byte_array[3]))
    info_packet.append(int(filesize_byte_array[2]))
    info_packet.append(int(filesize_byte_array[1]))
    info_packet.append(int(filesize_byte_array[0]))

    info_packet.append(FW_TYPE)

    fwType_byte_array = FW_CRC.to_bytes(2, byteorder='big')
    info_packet.append(int(fwType_byte_array[1]))
    info_packet.append(int(fwType_byte_array[0]))

    version_byte_array = Version.to_bytes(2, byteorder='big')
    info_packet.append(int(version_byte_array[1]))
    info_packet.append(int(version_byte_array[0]))

    crc16 = calculate_crc16(info_packet[1:])
    crc_byte_array = crc16.to_bytes(2, byteorder='big')
    info_packet.append(crc_byte_array[1])
    info_packet.append(crc_byte_array[0])
    info_packet.append(END_BYTE)
    print("header Packet : ", info_packet)
    port.write(bytes(info_packet))
    

def ota_check_response(port,cmd):

    while True:
        data = port.readall();
        if data == b'': continue
        resp =[]
        for i in range(0,len(data)):
            resp.append(int(data[i]))

        print("Response : ", resp)
        # check CRC
        #time.sleep(2)
        if resp[1] == cmd:
            if resp[4] == 0:
                return ACK
            else:
                return NACK

def ota_send_data(port, data, datalen,log):
    #port.write("Sending OTA Header".encode("utf-8"))
    print("Sending OTA DATA : ", datalen, len(data))
    fw_packet = []
    fw_packet.append(START_BYTE)
    fw_packet.append(CMD_FWDATA_PACKET)
    
    filesize_byte_array = datalen.to_bytes(4, byteorder='big')
    fw_packet.append(int(datalen & 0x00FF))
    fw_packet.append(int((datalen & 0xFF00) >> 8))
    

    for x in range(0,datalen):
        fw_packet.append(int(data[x]))
        val = int(data[x])
        fmt_data = f"{val},"
        log.write(fmt_data.encode("utf-8"));
    #print("w data : %x",data)

    crc16 = calculate_crc16(fw_packet[1:])
    crc_byte_array = crc16.to_bytes(2, byteorder='big')
    fw_packet.append(crc_byte_array[1])
    fw_packet.append(crc_byte_array[0])
    fw_packet.append(END_BYTE)

    #print("fw packet : ", fw_packet)
    port.write(bytes(fw_packet))
    log.write(b'\n');


def ota_send_stop_command(port):
    #port.write("Sending OTA START".encode("utf-8"))
    stop_packet = []
    stop_packet.append(START_BYTE)
    stop_packet.append(CMD_STOP_PACKET)
    stop_packet.append(CMD_STOP_PACKET_LENGTH)
    stop_packet.append(0x00)
    stop_packet.append(0x01)
    crc16 = calculate_crc16(stop_packet[1:])
    crc_byte_array = crc16.to_bytes(2, byteorder='big')
    stop_packet.append(crc_byte_array[1])
    stop_packet.append(crc_byte_array[0])
    stop_packet.append(END_BYTE)
    print("Stop Packet : ", stop_packet)
    #crc_byte_array = crc16.to_bytes(2, byteorder='big')
    port.write(bytes(stop_packet))



def main():
    # total arguments
    n = len(sys.argv)
    if(n < 3) : 
        print("Please enter filename and port")
    else:
            
        binfilePath = sys.argv[1]
        port = sys.argv[2]

        baud_rate = 115200  # Adjust this to match your device's baud rate

        try:
            # Open the serial port
            ser = serial.Serial(port, baud_rate, timeout=0.1)

            wfile = open("wfile.bin","wb")

            #open binary file
            #binfile_size = os.path.getsize(binfilePath)
            
            binfile = open(binfilePath,"rb")
            binfile_content = binfile.read()
            binfile_size = len(binfile_content)
            print("Binary file size : ",binfile_size)
            print(f"Serial port {port} is open.")

            # calculate crc of the fw
            fw_crc = calculate_crc16(binfile_content)
            #fw_crc = crc16(binfile_content)
            print("FW CRC : ", fw_crc)

            # Read and print data from the serial port
            #ser.flush()
            # start ota update
            ota_send_start_command(ser)
            resp = ota_check_response(ser,CMD_START_PACKET)
            if resp != ACK:
                print(ERROR_CODES[resp])
                return -1

            # send header command 
            ota_send_header_command(ser,binfile_size,FW_TYPE,fw_crc,FW_VERSION)
            resp = ota_check_response(ser,CMD_INFO_PACKET)
            if resp != ACK:
                print(ERROR_CODES[resp])
                return -1
            
            i=0
            #send firmware 
            print("updating firmware : ", 0 , "%" )
            while True:
                if binfile_size - i >= ETX_OTA_DATA_MAX_SIZE:
                    tobesend = binfile_content[i:i + ETX_OTA_DATA_MAX_SIZE]
                    ota_send_data(ser,tobesend,ETX_OTA_DATA_MAX_SIZE,wfile)
                    

                    resp = ota_check_response(ser,CMD_FWDATA_PACKET)
                    if resp == ACK:
                        i += ETX_OTA_DATA_MAX_SIZE
                    else:
                        print(ERROR_CODES[resp])
                        return -1

                    print("updating firmware : ", int(i/ binfile_size * 100) , "%" )
                elif binfile_size - i > 0:
                    tobesend = binfile_content[i:binfile_size]
                    ota_send_data(ser,tobesend,binfile_size - i,wfile)
                 

                    resp = ota_check_response(ser,CMD_FWDATA_PACKET)
                    if resp == ACK:
                        i = binfile_size
                        print("updating firmware : ", int(i/ binfile_size * 100)  , "%" )
                        
                    else:
                        print(ERROR_CODES[resp])
                        return -1

                if i == binfile_size:
                    print("Firmware update successfull!")
                    # send stop command
                    ota_send_stop_command(ser)
                    break
        
        except serial.SerialException as e:
            print(f"Error: {e}")
        finally:
            if ser.is_open:
                ser.close()
                binfile.close()
                wfile.close()
                print(f"Serial port {port} is closed.")



if __name__=="__main__":
    main()


