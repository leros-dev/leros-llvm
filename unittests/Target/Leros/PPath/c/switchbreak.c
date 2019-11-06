#include <stdlib.h>

int main() {
  int x = rand();
  switch(x) {
  case 200:
    x++;
    break;
  case 100:
    x--;
    break;
  default:
    x += 2;
    break;
  }
  return x;
}
