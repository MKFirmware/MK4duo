/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __NEXTION_H__
#define __NEXTION_H__

#if NEXTION_SERIAL > 0
  #if NEXTION_SERIAL == 1
    #define nexSerial Serial1
  #elif NEXTION_SERIAL == 2
    #define nexSerial Serial2
  #elif NEXTION_SERIAL == 3
    #define nexSerial Serial3
  #endif
#else
  #define nexSerial Serial1
#endif

#define NEX_RET_CMD_FINISHED                (0x01)
#define NEX_RET_EVENT_LAUNCHED              (0x88)
#define NEX_RET_EVENT_UPGRADED              (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD            (0x65)
#define NEX_RET_EVENT_POSITION_HEAD         (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD   (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD        (0x66)
#define NEX_RET_STRING_HEAD                 (0x70)
#define NEX_RET_NUMBER_HEAD                 (0x71)
#define NEX_RET_VALUE_HEAD                  (0x72)
#define NEX_RET_INVALID_CMD                 (0x00)
#define NEX_RET_INVALID_COMPONENT_ID        (0x02)
#define NEX_RET_INVALID_PAGE_ID             (0x03)
#define NEX_RET_INVALID_PICTURE_ID          (0x04)
#define NEX_RET_INVALID_FONT_ID             (0x05)
#define NEX_RET_INVALID_BAUD                (0x11)
#define NEX_RET_INVALID_VARIABLE            (0x1A)
#define NEX_RET_INVALID_OPERATION           (0x1B)

#define NEX_TIMEOUT                         100

/**
 * Push touch event occuring when your finger or pen coming to Nextion touch pannel. 
 */
#define NEX_EVENT_PUSH  (0x01)

/**
 * Pop touch event occuring when your finger or pen leaving from Nextion touch pannel. 
 */
#define NEX_EVENT_POP   (0x00)  

/**
 * Type of callback funciton when an touch event occurs. 
 * 
 * @param ptr - user pointer for any purpose. Commonly, it is a pointer to a object. 
 * @return none. 
 */
typedef void (*NexTouchEventCb)(void *ptr);

/**
 * Class NexObject
 */
class NexObject {

  public:

    /**
     * Constructor.
     *
     * @param pid - page id.
     * @param cid - component id.
     * @param name - pointer to an unique name in range of all components.
     */
    NexObject(uint8_t pid, uint8_t cid, const char *name);

    /**
     * iterate search pid, cid or event in list
     *
     */
    static void iterate(NexObject **list, const uint8_t pid, const uint8_t cid, const int32_t event);

    /**
     * Attach an callback function of push touch event.
     *
     * @param push - callback called with ptr when a push touch event occurs.
     * @param ptr - parameter passed into push[default:NULL].
     * @return none.
     *
     * @note If calling this method multiply, the last call is valid.
     */
    void attachPush(NexTouchEventCb push, void *ptr = NULL);

    /**
     * Detach an callback function.
     *
     * @return none.
     */
    void detachPush(void);

    /**
     * Attach an callback function of pop touch event.
     *
     * @param pop - callback called with ptr when a pop touch event occurs.
     * @param ptr - parameter passed into pop[default:NULL].
     * @return none.
     *
     * @note If calling this method multiply, the last call is valid.
     */
    void attachPop(NexTouchEventCb pop, void *ptr = NULL);

    /**
     * Detach an callback function.
     *
     * @return none.
     */
    void detachPop(void);

  public:

    /**
     * Show itself.
     */
    void show();

    /**
     * contorl timer enable.
     */
    void enable(const bool en=true);

    /**
     * Get text attribute of component.
     *
     * @param buffer - buffer storing text returned.
     * @param len - length of buffer.
     */
    void getText(char *buffer, uint16_t len, const char *pname=NULL);

    /**
     * Set text attribute of component.
     *
     * @param buffer - text buffer terminated with '\0'.
     * @param pname  - To set page name
     */
    void setText(const char *buffer, const char *pname=NULL);

    /**
     * Get val attribute of component
     *
     * @param number - buffer storing data return
     * @param pname  - To set page name
     */
    void getValue(uint32_t *number, const char *pname=NULL);

    /**
     * Set val attribute of component
     *
     * @param number - To set up the data
     * @param pname  - To set page name
     */
    void setValue(uint32_t number, const char *pname=NULL);

    /**
     * Add value to show.
     *
     * @param ch - channel of waveform(0-3).
     * @param number - the value of waveform.
     */
    void addValue(const uint8_t ch, const uint8_t number);

    /**
     * Get hig attribute of component
     *
     * @param number - buffer storing data return
     */
    void Get_cursor_height_hig(uint32_t *number);	

    /**
     * Set hig attribute of component
     *
     * @param number - To set up the data
     */
    void Set_cursor_height_hig(const uint32_t number);

    /**
     * Get maxval attribute of component
     *
     * @param number - buffer storing data return
     */
    void getMaxval(uint32_t *number);

    /**
     * Set maxval attribute of component
     *
     * @param number - To set up the data
     */
    void setMaxval(const uint32_t number);

    /**
     * Get minval attribute of component
     *
     * @param number - buffer storing data return
     */
    void getMinval(uint32_t *number);

    /**
     * Set minval attribute of component
     *
     * @param number - To set up the data
     */
    void setMinval(const uint32_t number);

    /**
     * Get bco attribute of component
     *
     * @param number - buffer storing data return
     */
    void Get_background_color_bco(uint32_t *number);

    /**
     * Set bco attribute of component
     *
     * @param number - To set up the data
     */
    void Set_background_color_bco(uint32_t number);

    /**
     * Get pco attribute of component
     *
     * @param number - buffer storing data return
     */
    void Get_font_color_pco(uint32_t *number); 

    /**
     * Set pco attribute of component
     *
     * @param number - To set up the data
     */
    void Set_font_color_pco(uint32_t number);			

    /**
     * Get xcen attribute of component
     *
     * @param number - buffer storing data return
     */
    void Get_place_xcen(uint32_t *number);

    /**
     * Set xcen attribute of component
     *
     * @param number - To set up the data
     */
    void Set_place_xcen(uint32_t number);

    /**
     * Get ycen attribute of component
     *
     * @param number - buffer storing data return
     */
    void Get_place_ycen(uint32_t *number);

    /**
     * Set ycen attribute of component
     *
     * @param number - To set up the data
     */
    void Set_place_ycen(uint32_t number);

    /**
     * Get font attribute of component
     *
     * @param number - buffer storing data return
     */
    void getFont(uint32_t *number);

    /**
     * Set font attribute of component
     *
     * @param number - To set up the data
     */
    void setFont(uint32_t number);

    /**
     * Get Crop pic attribute of component
     *
     * @param number - buffer storing data return
     */
    void getCropPic(uint32_t *number);	

    /**
     * Set Crop pic attribute of component
     *
     * @param number - To set up the data
     */
    void setCropPic(uint32_t number);

    /**
     * Get pic attribute of component
     *
     * @param number - buffer storing data return
     */
    void getPic(uint32_t *number);

    /**
     * Set pic attribute of component
     *
     * @param number - To set up the data
     */
    void setPic(uint32_t number);

    /**
     * Get componet vis.
     *
     * @return true if status show, false if status hide
     */
    bool getObjVis(void);

    /**
     * Set visibility attribute of component
     *
     * @param visible - To set visible or invisible
     */
    void SetVisibility(bool visible);

  protected:

    /**
     * Get componet page id.
     *
     * @return the id of page.
     */
    uint8_t getObjPid(void);

    /**
     * Get component id.
     *
     * @return the id of component.
     */
    uint8_t getObjCid(void);

    /**
     * Get component name.
     *
     * @return the name of component.
     */
    const char *getObjName(void);

  private:

    void push(void);
    void pop(void);

  private:

    uint8_t __pid;
    uint8_t __cid;
    const char *__name;
    bool __vis;

    NexTouchEventCb __cb_push;
    void *__cbpush_ptr;
    NexTouchEventCb __cb_pop;
    void *__cbpop_ptr;

};

/**
 * Class NexUpload
 */
#if HAS_SDSUPPORT

  class NexUpload {

    public:

      /**
       * Constructor.
       *
       * @param file_name - tft file name.
       * @upload_baudrate - set upload baudrate.
       */
      NexUpload(const char *file_name, uint32_t upload_baudrate);

      /**
       * Constructor.
       *
       * @param file_name - tft file name.
       * @upload_baudrate - set upload baudrate.
       */
      NexUpload(const String file_name, uint32_t upload_baudrate); 

      /**
       * destructor.
       */
      ~NexUpload(){}

      /**
       * start upload.
       */
      void startUpload(void);

    private:

      /**
       * get communicate baudrate.
       *
       * @return communicate baudrate.
       *
       */
      uint16_t _getBaudrate(void);

      /*
       * check tft file.
       *
       * @return true if success, false for failure.
       */
      bool _checkFile(void);

      /**
       * search communicate baudrate.
       *
       * @param baudrate - communicate baudrate.
       *
       * @return true if success, false for failure.
       */
      bool _searchBaudrate(uint32_t baudrate);

      /**
       * set upload baudrate.
       *
       * @param baudrate - set upload baudrate.
       *
       * @return true if success, false for failure.
       */
      bool _setUploadBaudrate(uint32_t baudrate);

      /**
       * start dowload tft file to nextion.
       *
       * @return none.
       */
      bool _uploadTftFile(void);

      /**
       * Receive string data.
       *
       * @param buffer - save string data.
       * @param timeout - set timeout time.
       * @param recv_flag - if recv_flag is true,will braak when receive 0x05.
       *
       * @return the length of string buffer.
       *
       */
      uint16_t recvRetString(String &string, uint32_t timeout = 100, bool recv_flag=false);

    private:

      uint32_t _baudrate; /*nextion serail baudrate*/
      const char *_file_name; /*nextion tft file name*/
      uint32_t _unuploadByte; /*unupload byte of tft file*/
      uint32_t _upload_baudrate; /*upload baudrate*/
    };

#endif // SDSUPPORT

//
// PUBBLIC FUNCTION
//

/**
 * Init Nextion.  
 * 
 * @return true if success, false for failure. 
 */
bool nexInit();

void getConnect(char *buffer, uint16_t len);

/**
 * Listen touch event and calling callbacks attached before.
 *
 * Supports push and pop at present.
 *
 * @param nex_listen_list - index to Nextion Components list.
 *
 * @warning This function must be called repeatedly to response touch events
 *  from Nextion touch panel. Actually, you should place it in your loop function.
 */
void nexLoop(NexObject *nex_listen_list[]);

void recvRetNumber(uint32_t *number);
void recvRetString(char *buffer, uint16_t len);
void sendCommand(const char* cmd);
void recvRetCommandFinished();

uint8_t Nextion_PageID();
void setCurrentBrightness(uint8_t dimValue);
void sendRefreshAll(void);

#endif /* __NEXTION_H__ */
