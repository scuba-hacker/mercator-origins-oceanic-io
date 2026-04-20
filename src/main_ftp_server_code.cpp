#ifdef BUILD_INCLUDE_FTP_SERVER_CODE


void _ftpConnectCallback(FtpOperation ftpOperation, uint32_t freeSpace, uint32_t totalSpace){
  switch (ftpOperation) {
    case FTP_CONNECT:
      USB_SERIAL_PRINTLN(F("FTP: Connected!"));
      break;
    case FTP_DISCONNECT:
      USB_SERIAL_PRINTLN(F("FTP: Disconnected!"));
      break;
    case FTP_FREE_SPACE_CHANGE:
      USB_SERIAL_PRINTF("FTP: Free space change, free %lu of %lu!\n", (unsigned long)freeSpace, (unsigned long)totalSpace);
      break;
    default:
      break;
  }
};

void _ftpTransferCallback(FtpTransferOperation ftpOperation, const char* name, uint32_t transferredSize){
  switch (ftpOperation) {
    case FTP_UPLOAD_START:
      USB_SERIAL_PRINTLN(F("FTP: Upload start!"));
      break;
    case FTP_UPLOAD:
      USB_SERIAL_PRINTF("FTP: Upload of file %s byte %lu\n", name, (unsigned long)transferredSize);
      break;
    case FTP_TRANSFER_STOP:
      USB_SERIAL_PRINTLN(F("FTP: Finish transfer!"));
      break;
    case FTP_TRANSFER_ERROR:
      USB_SERIAL_PRINTLN(F("FTP: Transfer error!"));
      break;
    default:
      break;
  }

  /* FTP_UPLOAD_START = 0,
   * FTP_UPLOAD = 1,
   *
   * FTP_DOWNLOAD_START = 2,
   * FTP_DOWNLOAD = 3,
   *
   * FTP_TRANSFER_STOP = 4,
   * FTP_DOWNLOAD_STOP = 4,
   * FTP_UPLOAD_STOP = 4,
   *
   * FTP_TRANSFER_ERROR = 5,
   * FTP_DOWNLOAD_ERROR = 5,
   * FTP_UPLOAD_ERROR = 5
   */
};

void setupFTPServer()
{
  /*
  A Patch has to be applied to the SimpleFTP library to make LittleFS the storage type for ESP32.
  patches/SimpleFTPServer_littlefs.py is executed in the pre-build-script.py to do this.
  Otherwise the SimpleFTP library would need to be forked only for this minor change.

  Filezilla needs to connect with Simple FTP, no TLS, and maximum connections =1
  Otherwise FileZilla will try to make two connections for a file transfer and it will
  fail at the waiting for welcome message.

  platform.io also has to have this set to make littlefs the file system to link in:

  board_build.filesystem = littlefs

  */
  if (LittleFS.begin(true)) 
  {
    ftpServer.setCallback(_ftpConnectCallback);
    ftpServer.setTransferCallback(_ftpTransferCallback);
    USB_SERIAL_PRINTLN("LittleFS opened!");
    ftpServer.begin("mercator","oceanic");    //username, password for ftp.   (default 21, 50009 for PASV)
    USB_SERIAL_PRINTLN("FTP Server Online");
    ftpActive = true;
  }
  else
  {
    USB_SERIAL_PRINTLN("LittleFS failed to open, FTP Server Offline");
    ftpActive = false;
  }
}

#endif
