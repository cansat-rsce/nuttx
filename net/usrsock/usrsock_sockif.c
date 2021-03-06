/****************************************************************************
 * net/usrsock/usrsock_sockif.c
 *
 *  Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *  Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_USRSOCK)

#include <sys/types.h>
#include <sys/socket.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "usrsock/usrsock.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int        usrsock_sockif_setup(FAR struct socket *psock, int protocol);
static sockcaps_t usrsock_sockif_sockcaps(FAR struct socket *psock);
static void       usrsock_sockif_addref(FAR struct socket *psock);
static int        usrsock_sockif_listen(FAR struct socket *psock, int backlog);
static int        usrsock_sockif_accept(FAR struct socket *psock,
                                        FAR struct sockaddr *addr,
                                        FAR socklen_t *addrlen,
                                        FAR struct socket *newsock);
static ssize_t    usrsock_sockif_send(FAR struct socket *psock,
                                      FAR const void *buf, size_t len,
                                      int flags);
static int        usrsock_sockif_close(FAR struct socket *psock);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_usrsock_sockif =
{
  usrsock_sockif_setup,       /* si_setup */
  usrsock_sockif_sockcaps,    /* si_sockcaps */
  usrsock_sockif_addref,      /* si_addref */
  usrsock_bind,               /* si_bind */
  usrsock_getsockname,        /* si_getsockname */
  usrsock_sockif_listen,      /* si_listen */
  usrsock_connect,            /* si_connect */
  usrsock_sockif_accept,      /* si_accept */
#ifndef CONFIG_DISABLE_POLL
  usrsock_poll,               /* si_poll */
#endif
  usrsock_sockif_send,        /* si_send */
  usrsock_sendto,             /* si_sendto */
#ifdef CONFIG_NET_SENDFILE
  NULL,                       /* si_sendfile */
#endif
  usrsock_recvfrom,           /* si_recvfrom */
  usrsock_sockif_close        /* si_close */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Parameters:
 *   psock    A pointer to a user allocated socket structure to be initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int usrsock_sockif_setup(FAR struct socket *psock, int protocol)
{
  int domain = psock->s_domain;
  int type = psock->s_type;
  int ret;

#ifdef CONFIG_NET_USRSOCK_NO_INET
  if (domain == PF_INET)
    {
      return -ENETDOWN;
    }
#endif

#ifdef CONFIG_NET_USRSOCK_NO_INET6
  if (domain == PF_INET6)
    {
      return -ENETDOWN;
    }
#endif

  if (domain == PF_INET || domain == PF_INET6)
    {
#ifndef CONFIG_NET_USRSOCK_UDP
      if (type == SOCK_DGRAM)
        {
          return -ENETDOWN;
        }
#endif
#ifndef CONFIG_NET_USRSOCK_TCP
      if (type == SOCK_STREAM)
        {
          return -ENETDOWN;
        }
#endif
    }
  else
    {
#ifndef CONFIG_NET_USRSOCK_OTHER
      return -ENETDOWN;
#endif
    }

  psock->s_type = PF_UNSPEC;
  psock->s_conn = NULL;

  /* Let the user socket logic handle the setup...
   *
   * A return value of zero means that the operation was
   * successfully handled by usrsock.  A negative value means that
   * an error occurred.  The special error value -ENETDOWN means
   * that usrsock daemon is not running.  The caller should attempt
   * to open socket with kernel networking stack in this case.
   */

  ret = usrsock_socket(domain, type, protocol, psock);
  if (ret == -ENETDOWN)
    {
      nwarn("WARNING: usrsock daemon is not running\n");
    }

  return ret;
}

/****************************************************************************
 * Name: usrsock_sockif_sockcaps
 *
 * Description:
 *   Return the bit encoded capabilities of this socket.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose capabilities are being
 *           queried.
 *
 * Returned Value:
 *   The non-negative set of socket cababilities is returned.
 *
 ****************************************************************************/

static sockcaps_t usrsock_sockif_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: usrsock_sockif_addref
 *
 * Description:
 *   Increment the reference count on the underlying connection structure.
 *
 * Parameters:
 *   psock - Socket structure of the socket whose reference count will be
 *           incremented.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void usrsock_sockif_addref(FAR struct socket *psock)
{
  FAR struct usrsock_conn_s *conn;

  DEBUGASSERT(psock != NULL && psock->s_conn != NULL);

  conn = psock->s_conn;
  DEBUGASSERT(conn->crefs > 0 && conn->crefs < 255);
  conn->crefs++;
}

/****************************************************************************
 * Name: usrsock_sockif_listen
 *
 * Description:
 *   To accept connections, a socket is first created with psock_socket(), a
 *   willingness to accept incoming connections and a queue limit for
 *   incoming connections are specified with psock_listen(), and then the
 *   connections are accepted with psock_accept().  For the case of AFINET
 *   and AFINET6 sockets, psock_listen() calls this function.  The
 *   psock_listen() call applies only to sockets of type SOCK_STREAM or
 *   SOCK_SEQPACKET.
 *
 * Parameters:
 *   psock    Reference to an internal, boound socket structure.
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, a negated errno value is
 *   returned.  See list() for the set of appropriate error values.
 *
 ****************************************************************************/

int usrsock_sockif_listen(FAR struct socket *psock, int backlog)
{
#warning "Missing logic for USRSOCK listen"

  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: usrsock_sockif_accept
 *
 * Description:
 *   The usrsock_sockif_accept function is used with connection-based socket
 *   types (SOCK_STREAM, SOCK_SEQPACKET and SOCK_RDM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an inet_accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   socket(), bound to a local address with bind(), and is listening for
 *   connections after a call to listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, inet_accept blocks the caller until a
 *   connection is present. If the socket is marked non-blocking and no
 *   pending connections are present on the queue, inet_accept returns
 *   EAGAIN.
 *
 * Parameters:
 *   psock    Reference to the listening socket structure
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *   newsock  Location to return the accepted socket information.
 *
 * Returned Value:
 *   Returns 0 (OK) on success.  On failure, it returns a negated errno
 *   value.  See accept() for a desrciption of the approriate error value.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int usrsock_sockif_accept(FAR struct socket *psock,
                                 FAR struct sockaddr *addr,
                                 FAR socklen_t *addrlen,
                                 FAR struct socket *newsock)
{
#warning "Missing logic for USRSOCK accept"

  return -EOPNOTSUPP;
}

/****************************************************************************
 * Name: usrsock_sockif_send
 *
 * Description:
 *   The usrsock_sockif_send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags (ignored)
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t usrsock_sockif_send(FAR struct socket *psock,
                                   FAR const void *buf,
                                   size_t len, int flags)
{
  return usrsock_sendto(psock, buf, len, flags, NULL, 0);
}

/****************************************************************************
 * Name: usrsock_sockif_close
 *
 * Description:
 *   Performs the close operation on an USRSOCK socket instance
 *
 * Parameters:
 *   psock   Socket instance
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int usrsock_sockif_close(FAR struct socket *psock)
{
  FAR struct usrsock_conn_s *conn = psock->s_conn;
  int ret;

  /* Perform some pre-close operations for the USRSOCK socket type. */

  /* Is this the last reference to the connection structure (there
   * could be more if the socket was dup'ed).
   */

  if (conn->crefs <= 1)
    {
      /* Yes... inform user-space daemon of socket close. */

      ret = usrsock_close(conn);

      /* Free the connection structure */

      conn->crefs = 0;
      usrsock_free(psock->s_conn);

      if (ret < 0)
        {
          /* Return with error code, but free resources. */

          nerr("ERROR: usrsock_close failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* No.. Just decrement the reference count */

      conn->crefs--;
    }

  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_USRSOCK */
