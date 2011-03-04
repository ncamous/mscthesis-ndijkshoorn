#include <VP_Os/vp_os_malloc.h>

#include <config.h>
#include <ardrone_tool/Com/config_com.h>

#include <arpa/inet.h>
#include <sys/types.h>
#include <ifaddrs.h>


vp_com_t* wifi_com(void)
{
  static vp_com_t com = {
    VP_COM_WIFI,
    FALSE,
    0,
#ifdef _WIN32
    { 0 },
#else // ! USE_MINGW32
    PTHREAD_MUTEX_INITIALIZER,
#endif // ! USE_MINGW32
    NULL,
    NULL,
    0,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
  };

  return &com;
}

vp_com_config_t* wifi_config(void)
{
  static vp_com_wifi_config_t config = {
    { 0 },
    WIFI_MOBILE_IP,
    WIFI_NETMASK,
    WIFI_BROADCAST,
    WIFI_GATEWAY,
    WIFI_SERVER,
    WIFI_INFRASTRUCTURE,
    WIFI_SECURE,
    WIFI_PASSKEY
  };

  struct ifaddrs * ifAddrStructHead = NULL;
  struct ifaddrs * ifAddrStruct = NULL;
  struct in_addr tmpAddr;

  getifaddrs(&ifAddrStruct);
  ifAddrStructHead = ifAddrStruct;

  while (ifAddrStruct != NULL)
  {
    // Looking for WIFI interface's IP address corresponding to WIFI_BASE_ADDR
    if (ifAddrStruct->ifa_addr->sa_family == AF_INET) {
      tmpAddr = ((struct sockaddr_in *)ifAddrStruct->ifa_addr)->sin_addr;
      if ( (ntohl(tmpAddr.s_addr) & 0xFFFFFF00) == WIFI_BASE_ADDR ) {
        inet_ntop(AF_INET, &tmpAddr, config.localHost, VP_COM_NAME_MAXSIZE);
      }
    }
    ifAddrStruct = ifAddrStruct->ifa_next;
  }

  if (ifAddrStructHead != NULL) {
    freeifaddrs(ifAddrStructHead);
  }

  return (vp_com_config_t*) &config;
}

vp_com_connection_t* wifi_connection(void)
{
  static vp_com_wifi_connection_t connection = {
    0,
    WIFI_NETWORK_NAME
  };

  return (vp_com_connection_t*) &connection;
}

void wifi_config_socket(vp_com_socket_t* socket, VP_COM_SOCKET_TYPE type, int32_t port, const char* serverhost)
{
  vp_os_memset(socket, 0, sizeof(vp_com_socket_t));

  socket->type           = type;
  socket->protocol       = VP_COM_TCP;
  socket->port           = port;

  if(serverhost && socket->type == VP_COM_CLIENT)
    strcpy(socket->serverHost, serverhost);
}
