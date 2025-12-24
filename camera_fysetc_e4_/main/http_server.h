/**
 * @file http_server.h
 * @brief HTTP server for web-based motor control
 */

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <stdbool.h>

/**
 * @brief Start HTTP server
 * @return true on success
 */
bool http_server_start(void);

/**
 * @brief Stop HTTP server
 */
void http_server_stop(void);

#endif // HTTP_SERVER_H

