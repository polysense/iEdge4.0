/**
 * @file  pos_net.h  
 * @brief Netowork protocol stack common library
 * @author Runby F.
 * @date 2022-2-22
 * @copyright Polysense
 */

#ifndef __POS_NET_H__
#define __POS_NET_H__

/**
 * @brief U16 network order tranlation
 */
#define POS_NTOHS(value)           ((((value)&0xff)<<8)|(((value)>>8)&0xff))

/**
 * @brief U32 network order tranlation
 */
#define POS_NTOHL(value)           ((((value)&0xff)<<24)|(((value)&0xff00)<<8)|(((value)>>24)&0xff)|(((value)>>8)&0xff00))

/**
 * @brief MQTT buffer length of bytes
 */
#define POS_NET_MQTT_BUF_SIZE 1280

/**
 * @brief Transportation protocol define
 */
typedef enum {
  NET_TRANS_UDP = 0, ///< UDP
  NET_TRANS_TCP = 1, ///< TCP
} pos_net_trans_t;

/**
 * @brief External module running status define
 */
typedef enum {
  RUN_LEVEL_POWER_OFF = 0, ///< Power off
  RUN_LEVEL_POWER_SAVING = 1, ///< Power saving
  RUN_LEVEL_NORMAL = 2, ///< Normal working
  RUN_LEVEL_RSVD, ///< Reserved
} pos_run_level_t;

/**
 * @brief MQTT message header structure
 */
typedef struct {
    pos_u8_t qos; ///< MQTT QOS
    pos_u8_t retained; ///< MQTT retained flag
    pos_u8_t dup; ///< MQTT duplicate flag
    pos_u8_t rsvd; ///< Reserved
    pos_u16_t id; ///< MQTT message id
    pos_u16_t rsvd2;  ///< Reserved#2
    void *payload; ///< Data payload
    pos_size_t payloadlen; ///< Data payload length
} pos_mqtt_message_t;

/**
 * @brief MQTT message and topic structure
 */
typedef struct {
    pos_mqtt_message_t  *message; ///< MQTT message
    pos_c_lstring_t  *topic; ///< MQTT topic
} pos_mqtt_data_t;

/**
 * @brief MQTT message callback function type
 */
typedef void (*pos_mqtt_message_handler_t)(pos_mqtt_data_t*);

/**
 * @brief MQTT handle type
 */
typedef void * pos_net_mqtt_handle_t;

/**
 * @brief External network module library
 */
typedef struct {
  /**
   * @brief Init external module
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */
  pos_status_t (*init)(void);

  /**
   * @brief Set external module working level
   * @param[in] level Working level, refer to @ref pos_run_level_t
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */  
  pos_status_t (*run)(pos_run_level_t level);

  /**
   * @brief Check network ready
   * @return     0: Successful (network is ready)\n
                !=0: Failed (network is not ready), refer to @ref pos_status_t
   */
  pos_status_t (*ready)(pos_u32_t timeout_ms);  

  /**
   * @brief Create a socket
   * @param[out] p_sock Return the corresponding socket ID
   * @param[in] trans Transportatio protocol, refer to @ref pos_net_trans_t
   * @param[in] port Transport port in host order (0: any port, others: specified port number)
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */  
  pos_status_t (*open)(pos_u32_t *p_sock, pos_net_trans_t trans, pos_u16_t port);

  /**
   * @brief Close a socket
   * @param[in] sn Socket ID
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */  
  pos_status_t (*close)(pos_u32_t sn);

  /**
   * @brief Connect a socket to given IP/port
   * @param[in] sn Socket ID
   * @param[in] ip Target IP address in network order (for example: 1.2.3.4 ip={1,2,3,4})
   * @param[in] port Target port in host order
   * @return     0: Successful\n
                !=0: Failed, refer to @ref pos_status_t
   */  
  pos_status_t (*connect)(pos_u32_t sn, pos_u8_t * ip, pos_u16_t port);

  /**
   * @brief Send data
   * @param[in] sn Socket ID
   * @param[in] buf Data buffer to be sent
   * @param[in] len Data buffer length of bytes
   * @param[in] ip Target IP address in network order  (for example: 1.2.3.4 ip={1,2,3,4})
   * @param[in] port Target port in host order
   * @return >0: actual sending length\n
             =0: network busy\n
             <0: network error
   * @note ip/port is only used for TCP protocol or some special modules
   */    
  pos_i32_t    (*sendto)(pos_u32_t sn, void * buf, pos_size_t len, pos_u8_t *ip, pos_u16_t port); 

  /**
   * @brief Receive data
   * @param[in] sn Socket ID
   * @param[in] buf Data buffer for receiving
   * @param[in] len Expect receiving length of bytes
   * @param[out] ip Return the remote IP address in network order
   * @param[out] port Return the remote port in host order
   * @return >=0: actual receiving length\n
             <0: network error
   * @note ip/port is only available for some special modules
   */      
  pos_i32_t    (*recvfrom)(pos_u32_t sn, void * buf, pos_size_t len, pos_u8_t *ip, pos_u16_t *port, pos_u32_t timeout_ms); /* use timeout=0 for none blocking mode */

  /**
   * @brief Reserved
   */    
  pos_u32_t   rsvd[4]; 
} pos_net_t;

/**
 * @brief Network protocol stack library
 */ 
typedef struct {
  /**
  * @brief U16 array network byte order translation
  * @param[in] data Data array to be translated
  * @param[in] num Number of U16
  */
  void (*ntohs)(pos_u16_t *data, pos_size_t num);

  /**
  * @brief U32 array network byte order translation
  * @param[in] data Data array to be translated
  * @param[in] num Number of U32
  */  
  void (*ntohl)(pos_u32_t *data, pos_size_t num);

  /**
  * @brief Translate IP string to binary bytes
  * @param[in] ips IP string
  * @param[in] ipu8 Translated binary IP array in network order
  */  
  void (*str2ipu8)(const char *ips, pos_u8_t *ipu8);

  /**
  * @brief DNS resolve
  * @param[in] net External module library
  * @param[in] dns DNS server IP in network order
  * @param[in] url String of URL (or IP address)
  * @param[out] ip_nwk Resolved IP address in network order
  * @param[out] ttl DNS TTL time
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */
  pos_status_t (*dns_resolve)(pos_net_t *net, pos_u8_t *dns, const char *url, pos_u8_t *ip_nwk, pos_u32_t *ttl);

  /**
  * @brief Allocate a new MQTT handle
  * @param[in] net External module library
  * @return   NOT NULL: The new allocated mqtt handle\n
              NULL: Allocate failed
  */
  pos_net_mqtt_handle_t (*mqtt_alloc)(pos_net_t *net);

  /**
  * @brief Release a MQTT 
  * @param[in] h MQTT handle to be released
  */
  void (*mqtt_free)(pos_net_mqtt_handle_t h);

  /**
  * @brief Connect MQTT server
  * @param[in] h MQTT handle
  * @param[in] ip Server IP address in network order
  * @param[in] port Server port in host order
  * @param[in] mcid String for MQTT Client ID
  * @param[in] musr String for MQTT Client username
  * @param[in] mpwd String for MQTT Client password
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */
  pos_status_t (*mqtt_connect)(pos_net_mqtt_handle_t h, pos_u8_t *ip, pos_u16_t port, const char *mcid, const char *musr, const char *mpwd);

  /**
  * @brief Disconnect from MQTT server
  * @param[in] h MQTT handle
  */
  void (*mqtt_disconnect)(pos_net_mqtt_handle_t h);

  /**
  * @brief MQTT Subscribe
  * @param[in] h MQTT handle
  * @param[in] topic MQTT topic to be subscribed
  * @param[in] qos MQTT QOS
  * @param[in] mqtt_cb MQTT message callback function
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */
  pos_status_t (*mqtt_subscribe)(pos_net_mqtt_handle_t h, const char *topic, pos_u8_t qos, pos_mqtt_message_handler_t mqtt_cb);

  /**
  * @brief MQTT Publish
  * @param[in] h MQTT handle
  * @param[in] topic MQTT topic to be published
  * @param[in] qos MQTT QOS
  * @param[in] payload Data buffer for publishing
  * @param[in] len Bytes length of data buffer
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  */
  pos_status_t (*mqtt_publish)(pos_net_mqtt_handle_t h, const char *topic, pos_u8_t qos, void *payload, pos_size_t len);

  /**
  * @brief MQTT yielding 
  * @param[in] h MQTT handle
  * @param[in] timeout_ms This operation's timeout time in unit of ms
  * @return     0: Successful\n
               !=0: Failed, refer to @ref pos_status_t
  * @note MQTT protocol stack itself does NOT occupy any standalone tastk (thread). 
          It's necessary to call this yield function to make the MQTT keep-alvie if longterm MQTT connection is used.
  */ 
  pos_status_t (*mqtt_yield)(pos_net_mqtt_handle_t h, pos_u32_t timeout_ms);

} pos_lib_nps_t;

#endif
