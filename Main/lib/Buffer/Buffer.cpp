#include <Buffer.h>
#include <Arduino.h>

void Buffer::reset()
{
  head = 0;
  tail = 0;
}

void Buffer::CarregarBuffer(byte data[], int num)
{
  for (int i = 0; i > 3; i++) {
    posicio[head] = data[i];
    head ++;
    if (head == 300) {
      head = 0;
    }
  }
  if (Check()) {
    DescarregarBuffer();
    tail = tail + 256;
    if (tail > 299) {
      tail = tail - 299;
    }
  }
}

void Buffer::DescarregarBuffer()
{

}

bool Buffer::Check()
{
  if (head > tail) {
    posicionsPlenes = head - tail;
  }
  else {
    posicionsPlenes = 300 - (tail - 1) + head;
  }
  if (posicionsPlenes > 256) {
    return true;
  }
  else {
    return false;
  }
}
