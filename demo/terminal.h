#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <Windows.h>

void cls(void) {
  HANDLE hOutput = GetStdHandle(STD_OUTPUT_HANDLE);
  COORD topLeft = {0, 0};
  DWORD dwCount, dwSize;
  CONSOLE_SCREEN_BUFFER_INFO csbi;
  GetConsoleScreenBufferInfo(hOutput, &csbi);
  dwSize = csbi.dwSize.X * csbi.dwSize.Y;
  FillConsoleOutputCharacter(hOutput, 0x20, dwSize, topLeft, &dwCount);
  FillConsoleOutputAttribute(hOutput, 0x07, dwSize, topLeft, &dwCount);
  SetConsoleCursorPosition(hOutput, topLeft);
}

void get_terminal_size(int& width, int& height) {
  CONSOLE_SCREEN_BUFFER_INFO csbi;
  GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
  width = (int)(csbi.dwSize.X);
  height = (int)(csbi.dwSize.Y);
}

void hidecursor() {
  HANDLE consoleHandle = GetStdHandle(STD_OUTPUT_HANDLE);
  CONSOLE_CURSOR_INFO info;
  info.dwSize = 100;
  info.bVisible = FALSE;
  SetConsoleCursorInfo(consoleHandle, &info);
}
#endif /* _WIN32 */

#ifdef __unix__
#include <stdio.h>
#include <sys/ioctl.h>
void cls(void) { printf("\x1B[2J"); }

void get_terminal_size(int& width, int& height) {
  struct winsize w;
  ioctl(fileno(stdout), TIOCGWINSZ, &w);
  width = (int)(w.ws_col);
  height = (int)(w.ws_row);
}

void hidecursor() { printf("\e[?25l"); }

void showcursor() { printf("\e[?25h"); }

#endif /* __unix__ */
