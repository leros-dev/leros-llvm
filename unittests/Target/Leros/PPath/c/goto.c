#include <stdlib.h>

int main() {
  int x = rand();
  int y = 0;
back:  
  x++;
  if (x > 50) goto back;
  return x;
}
