#include <sys/ioctl.h>
#include <cstdio>
#include <cunistd>

namespace Tran {

static void
terminalSize(size_t& width, size_t& height)
{
  struct winsize w;
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
  width = w.ws_col;
  height = w.ws_row;
}

}