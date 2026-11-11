#pragma once

#include <cerrno>
#include <cstring>
#include <poll.h>
#include <sys/socket.h>

namespace devourer {

inline bool pcie_guard_send(int socket, int container, int group, int device) {
  int fds[] = {container, group, device};
  char byte = 'F';
  iovec data{&byte, 1};
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(fds))]{};
  msghdr message{};
  message.msg_iov = &data;
  message.msg_iovlen = 1;
  message.msg_control = control;
  message.msg_controllen = sizeof(control);
  auto* cmsg = CMSG_FIRSTHDR(&message);
  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SCM_RIGHTS;
  cmsg->cmsg_len = CMSG_LEN(sizeof(fds));
  std::memcpy(CMSG_DATA(cmsg), fds, sizeof(fds));
  if (sendmsg(socket, &message, MSG_NOSIGNAL) != 1) return false;
  pollfd ready{socket, POLLIN, 0};
  int result;
  do { result = poll(&ready, 1, 5000); } while (result < 0 && errno == EINTR);
  return result == 1 && recv(socket, &byte, 1, 0) == 1 && byte == 'A';
}

}  // namespace devourer
